"""Free-body 3D wheel-leg simulation, with the actual recovered MCU filters.

The wheel drive law and mass distribution are explicit unmeasured assumptions.
No stabilizing force is applied to the chassis pitch/roll/yaw.
"""
from collections import deque
from pathlib import Path
import math
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
from .firmware import Firmware

ROOT=Path(__file__).resolve().parents[1]
OBS_MEAN=np.array([0]*15+[.012,.006,0],np.float32)
OBS_STD=np.array([.2,.2,3,3,3,.3,.3,.15,.15,.21,1,1,.15,1,1,.004,.002,.05],np.float32)
RADIUS=.0308  # Measured STEP tyre diameter is 61.6mm.
ENCODER_RADIUS=.0275  # The recovered MCU still decodes a 55mm wheel.
# Contract 5 adds two leg-extension offset channels to the two wheel channels.
# Training curriculum knob for the measured floor lag. 0 reproduces the
# nominal actuator model; 1 reproduces the measured 20-60ms drive delay plus
# 35-85ms motor and feedback lag. train.py raises this during learning.
LAG_SEVERITY=1.0
# Real wheel drivers accept a speed setpoint and close the loop
# internally. "speed" is the measured model; "torque" preserves the old
# voltage/back-EMF model for reproducing historical runs.
WHEEL_ACTUATOR="speed"
# Mix between the historical torque/back-EMF model (0) and an ideal
# speed servo (1). Real drivers close a finite-bandwidth speed loop; this
# blend is calibrated against the measured floor oscillation.
WHEEL_SPEED_BLEND=0.7
# Fore-aft centre-of-mass offset used to calibrate the measured robot pose.
COM_X_SHIFT=0.014  # measured balance pose: the real robot settles near -11 deg
COM_Y_SHIFT=0.0
# Optional mechanical vibration injected into the gyro during diagnostics.
VIBRATION_AMP=0.0
VIBRATION_HZ=12.0
LEG_ACTION_SCALE=.005    # metres of extension target per normalized action unit
LEG_SLEW_PER_SECOND=10.0 # normalized leg action units per second, matches firmware
LEG_TAU_DEFAULT=.02      # nominal first-order servo tracking lag, seconds


def quaternion(roll,pitch,yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]


class RiderEnv(gym.Env):
    metadata={'render_modes':['rgb_array'],'render_fps':50}
    def __init__(self,domain=0.0,commands=True,pushes=True,episode_seconds=12,render_mode=None,seed=None,native_policy=None,profile=None,yaw_integral_gain=0.,yaw_integral_limit=.05,command_interval=(1.5,3.),physics='legacy'):
        if physics not in ('legacy','measured'):raise ValueError('Unknown physics configuration')
        stem='rider_measured' if physics=='measured' else 'rider'
        path=ROOT/f'models/cad/{stem}_cad.xml' if render_mode and (ROOT/f'models/cad/{stem}_cad.xml').exists() else ROOT/f'models/{stem}.xml'
        self.model=mujoco.MjModel.from_xml_path(str(path))
        self.physics=physics;self.calibration=None;self.physics_sample=None
        if physics=='measured':
            from .calibration import MeasuredCalibration
            self.calibration=MeasuredCalibration(self.model)
        self.data=mujoco.MjData(self.model);self.firmware=Firmware()
        self.action_space=spaces.Box(-1,1,(4,),np.float32)
        self.observation_space=spaces.Box(-5,5,(18,),np.float32)
        self.domain=float(domain);self.commands=commands;self.pushes=pushes;self.episode_seconds=episode_seconds
        self.render_mode=render_mode;self.renderer=None
        self.native_policy=native_policy
        self.profile=profile
        self.wheel_actuator=WHEEL_ACTUATOR
        self.vibration_amp=VIBRATION_AMP;self.vibration_hz=VIBRATION_HZ
        self.yaw_integral_gain=yaw_integral_gain;self.yaw_integral_limit=yaw_integral_limit
        self.command_interval=command_interval
        self.base=self.model.body('base').id;self.ground=self.model.geom('ground').id
        self.wheel_bodies=[self.model.body(x).id for x in ('wheel11','wheel21')]
        self.body_velocity=np.zeros(6)
        self.wheel_q=np.array([self.model.jnt_qposadr[self.model.joint(x).id] for x in ('wheel11','wheel21')])
        self.wheel_v=np.array([self.model.jnt_dofadr[self.model.joint(x).id] for x in ('wheel11','wheel21')])
        self.leg_q=np.array([self.model.jnt_qposadr[self.model.joint(x).id] for x in ('extension12','extension22')])
        self.leg_v=np.array([self.model.jnt_dofadr[self.model.joint(x).id] for x in ('extension12','extension22')])
        self.nominal_mass=self.model.body_mass.copy();self.nominal_inertia=self.model.body_inertia.copy();self.nominal_ipos=self.model.body_ipos.copy()
        self.reset(seed=seed)

    def _angles(self):
        r=self.data.xmat[self.base].reshape(3,3)
        return math.atan2(r[2,1],r[2,2]),math.asin(float(np.clip(-r[2,0],-1,1))),math.atan2(r[1,0],r[0,0])

    def _velocity(self):
        r=self.data.xmat[self.base].reshape(3,3);heading=math.atan2(r[1,0],r[0,0])
        return self.data.qvel[0]*math.cos(heading)+self.data.qvel[1]*math.sin(heading)

    def axle_velocity(self):
        heading=self._angles()[2];velocity=0.
        for body in self.wheel_bodies:
            mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,body,self.body_velocity,0)
            velocity+=self.body_velocity[3]*math.cos(heading)+self.body_velocity[4]*math.sin(heading)
        return velocity*.5

    def _read_feedback(self,slot):
        if slot<8:
            i=slot%2;direction=1 if i==0 else -1
            speed=round(float(self.data.qvel[self.wheel_v[i]])*1024/(2*math.pi*4.8)*self.encoder_scale)*direction
            if self.wheel_feedback_noise:speed=round(speed+float(self.np_random.normal(0,self.wheel_feedback_noise)))
            angle=float(self.data.qpos[self.wheel_q[i]])/(2*math.pi)*1024*self.encoder_scale
            position=math.floor(angle*direction)%1024
            self.firmware.feedback((11,21)[i],position,speed,self.us,self.feedback)
        else:
            i=slot-8;extension=float(self.data.qpos[self.leg_q[i]])+.063
            ticks=round((1.1408376963350786-math.acos(float(np.clip(extension/.096,-1,1))))/(2*math.pi)*1024)
            position=(610-ticks) if i==0 else (431+ticks)
            if self.leg_feedback_noise:
                position=(position+int(round(float(self.np_random.normal(0,self.leg_feedback_noise)))))%1024
            self.firmware.feedback((12,22)[i],position,0,self.us,self.feedback)

    def _control(self):
        self.out=self.firmware.control(self.feedback,self.command,self.us).copy()
        if self.ticks:self.odom_reference+=float(self.out[10])*self.control_ticks*.002
        if self.native_policy:self.native_policy.tick(self)

    def _apply_action(self):
        dt=self.control_ticks*.002
        if self.native_policy:
            self.applied[:]=self.native_policy.wheels
            self.yaw_integral=float(self.native_policy.yaw_integral)
            self.delay.append((self.us+self.drive_delay_us,np.array([self.applied[0],-self.applied[1]])/1023))
            leg_action=np.asarray(self.native_policy.commands[2:4],np.float32)
        else:
            if self.teacher_mode=='firmware':target=np.clip(self.out[14:16],-1023,1023)
            else:target=self.action[:2]*1023
            step=.004 if self.first_control else dt
            self.yaw_integral=float(np.clip(self.yaw_integral+self.yaw_integral_gain*(self.out[11]-self.out[5])*step,
                -self.yaw_integral_limit,self.yaw_integral_limit))
            target=np.clip(target-self.yaw_integral*1023,-1023,1023)
            delta=80 if self.first_control else 20000*step
            self.applied+=np.clip(target-self.applied,-delta,delta)
            self.first_control=False
            # std::lround rounds halves away from zero.
            command=np.sign(self.applied)*np.floor(np.abs(self.applied)+.5)
            self.delay.append((self.us+self.drive_delay_us,np.array([command[0],-command[1]])/1023))
            leg_delta=LEG_SLEW_PER_SECOND*dt
            self.leg_action_applied+=np.clip(self.action[2:4]-self.leg_action_applied,-leg_delta,leg_delta)
            leg_action=self.leg_action_applied
        # Servo target model: first-order tracking lag plus linkage backlash.
        base=np.asarray(self.out[16:18],np.float32)
        target=base+np.asarray(leg_action,np.float32)*LEG_ACTION_SCALE
        self.leg_servo+=(target-self.leg_servo)*(1-math.exp(-dt/self.leg_tau))
        for i in range(2):
            if self.leg_servo[i]>self.leg_hyst[i]+self.leg_backlash:self.leg_hyst[i]=self.leg_servo[i]-self.leg_backlash
            elif self.leg_servo[i]<self.leg_hyst[i]-self.leg_backlash:self.leg_hyst[i]=self.leg_servo[i]+self.leg_backlash
        self.data.ctrl[2:]=np.clip(self.leg_hyst-.063,-.018,.018)

    def _observation(self,dt):
        v=self.out
        odom=self.encoder_position()
        self.raw_observation=np.array([v[0]*math.pi/180,v[1]*math.pi/180,v[3],v[4],v[5],
            v[6]*RADIUS/ENCODER_RADIUS,v[7]*RADIUS/ENCODER_RADIUS,v[8],v[9],v[10],v[11],v[12],
            odom-self.odom_reference,np.sign(self.applied[0])*np.floor(abs(self.applied[0])+.5)/1023,
            np.sign(self.applied[1])*np.floor(abs(self.applied[1])+.5)/1023,dt,
            .004 if self.ticks==0 else self.control_ticks*.002,self.yaw_integral],np.float32)
        return np.clip((self.raw_observation-OBS_MEAN)/OBS_STD,-5,5).astype(np.float32)

    def encoder_position(self):
        # Same integer encoder odometry and legacy pi constant as control::wheelDistance.
        return np.float32(np.float32(float(self.feedback[2]+self.feedback[3])*.5/1024*3.1415*.055)*np.float32(61.6/55))

    def reset(self,seed=None,options=None):
        super().reset(seed=seed);options=options or {};rng=self.np_random;d=self.domain
        if self.calibration:
            self.physics_sample=self.calibration.apply(self.model,rng,d)
            self.supported_mass=self.physics_sample['fixed_chassis_mass_kg']
        else:
            self.model.body_mass[:]=self.nominal_mass*(1+rng.uniform(-.18,.18)*d)
            self.model.body_inertia[:]=self.nominal_inertia*(1+rng.uniform(-.25,.25)*d)
            self.model.body_ipos[:]=self.nominal_ipos
            self.model.body_ipos[self.base,:]+=rng.uniform([-0.003,-0.001,-0.008],[.003,.001,.008])*d
            self.supported_mass=self.model.body_mass[self.base]
        # Apply the calibrated fore-aft CoM offset after both the measured
        # calibration path and the generic randomization have set body_ipos.
        self.model.body_ipos[self.base,0]+=COM_X_SHIFT
        self.model.body_ipos[self.base,1]+=COM_Y_SHIFT
        self.model.geom_friction[:,0]=rng.uniform(.55,1.15) if d else .9
        self.torque_limit=.13*(1+rng.uniform(-.25,.25)*d)
        speed_draw=rng.uniform(0,1)
        # Official no-load speed is 400rpm. Randomize speed itself by +/-30%.
        self.back_emf=1/(41.89*(1+(-.3+.6*speed_draw)*d))
        if self.profile=='legacy_extreme':
            # Earlier experiments reached 2000rpm; retain that as a stress test.
            self.back_emf=(1+(-.8+1.4*speed_draw)*d)/41.89
        self.motor_tau=.008+rng.uniform(0,.032)*d;self.encoder_scale=1+rng.uniform(-.04,.04)*d
        # Soft wheel-speed servo: the driver's closed loop has a finite
        # response time (speed_tau). The torque needed to follow that ramp is
        # J_eff * dv/dt, clamped by the motor torque limit; external load then
        # perturbs the achieved speed like the real wheel/body contact.
        self.speed_tau=.02;self.wheel_eff_inertia=1.2e-4
        self.drive_delay_us=int(rng.uniform(0,18000)*d)
        self.gyro_bias=rng.uniform(-.2,.2,3)*d
        self.accel_bias=rng.uniform(-.025,.025,3)*d
        self.control_ticks=int(rng.choice([2,3,4])) if d else 3
        self.feedback_ticks=int(rng.choice([1,2,3,4,5])) if d else 2
        self.wheel_feedback_noise=abs(rng.uniform(0,3))*d
        self.leg_feedback_noise=abs(rng.uniform(0,1.5))*d
        self.leg_tau=max(.006,LEG_TAU_DEFAULT+float(rng.uniform(-.014,.045))*d)
        self.leg_backlash=abs(float(rng.uniform(0,.0012)))*d
        if d and self.profile=='lag':
            self.control_ticks=4;self.feedback_ticks=5
            self.motor_tau=rng.uniform(.028,.042);self.drive_delay_us=int(rng.uniform(12000,18000))
            self.leg_tau=rng.uniform(.05,.08)
        elif d and self.profile=='fast':
            self.back_emf=1/(41.89*rng.uniform(1.1,1.3));self.motor_tau=rng.uniform(.012,.024)
        elif d and self.profile=='realistic':
            # Measured floor behaviour: command->speed response peaked around
            # 55-90 ms of total lag, with a slow/quantised bus feedback. The
            # curriculum severity blends this into the nominal actuator model.
            sev=max(0.,min(1.,LAG_SEVERITY))
            # In speed-servo mode the driver itself closes the wheel loop, so
            # the floor lag is command delay + feedback rate, not a huge motor
            # torque time constant (that belonged to the old torque model).
            real_tau=rng.uniform(.02,.07);real_delay=rng.uniform(15000,50000)
            real_fb=int(rng.choice([4,5]));real_leg=rng.uniform(.03,.08)
            self.speed_tau=self.speed_tau+(real_tau-self.speed_tau)*sev
            self.drive_delay_us=int(self.drive_delay_us+(real_delay-self.drive_delay_us)*sev)
            if rng.random()<sev:self.feedback_ticks=real_fb
            self.leg_tau=self.leg_tau+(real_leg-self.leg_tau)*sev
            if sev>.5:self.control_ticks=int(rng.choice([3,4]))
            self.leg_backlash=abs(float(rng.uniform(0,.0015)))*sev
            self.motor_tau=min(self.motor_tau,.012)
        elif d and self.profile=='corner':
            self.control_ticks=int(rng.choice([2,3,4]));self.feedback_ticks=int(rng.choice([1,2,3,4,5]))
            self.motor_tau=rng.uniform(.012,.032);self.drive_delay_us=int(rng.uniform(8000,18000))
            self.leg_tau=rng.uniform(.02,.08);self.leg_backlash=rng.uniform(0,.0015)
            speed=rng.uniform(1.18,1.3) if rng.random()<.5 else rng.uniform(.7,.85)
            self.back_emf=1/(41.89*speed);self.torque_limit=.13*rng.uniform(1.05,1.25)
            if not self.calibration:
                self.model.body_ipos[self.base,0]=self.nominal_ipos[self.base,0]+rng.choice([-1,1])*rng.uniform(.0015,.003)
                self.model.body_ipos[self.base,2]=self.nominal_ipos[self.base,2]+rng.uniform(.004,.008)
                self.model.body_inertia[:]=self.nominal_inertia*rng.uniform(.75,1.)
        self.policy_ticks=math.ceil(5/self.control_ticks)*self.control_ticks
        self.push_force=rng.uniform(.1,.8)*(0.4+.6*d)
        self.push_roll_torque=rng.uniform(.004,.02)*(0.4+.6*d)
        self.push_pitch_torque=rng.uniform(.01,.04)*(0.4+.6*d)
        self.model.geom_quat[self.ground]=quaternion(0,rng.uniform(-.035,.035)*d,0)
        mujoco.mj_setConst(self.model,self.data);mujoco.mj_resetData(self.model,self.data)
        pitch=options.get('pitch',rng.uniform(-.06,.06));roll=options.get('roll',rng.uniform(-.012,.012))
        self.data.qpos[3:7]=quaternion(roll,pitch,options.get('yaw',rng.uniform(-math.pi,math.pi)))
        self.data.qpos[self.leg_q]=-.018;self.data.ctrl[2:]=-.018
        self.data.qpos[2]=.0905;mujoco.mj_forward(self.model,self.data)
        z=min(self.data.xpos[self.model.body(x).id,2] for x in ('wheel11','wheel21'))
        self.data.qpos[2]+=RADIUS-z-.0002
        self.data.qfrc_applied[self.leg_v]=self.supported_mass*9.81*.5
        mujoco.mj_forward(self.model,self.data)
        self.us=self.ticks=0;self.firmware.reset(pitch*180/math.pi,roll*180/math.pi)
        self.feedback=np.array([0,0,0,0,610,431,0,0],np.float32)
        for slot in range(10):self._read_feedback(slot)
        self.command=np.array([128,128,128],np.uint8);self.action=np.zeros(4,np.float32)
        self.applied=np.zeros(2);self.drive=np.zeros(2);self.torque=np.zeros(2);self.delay=deque()
        self.wheel_integral=np.zeros(2)
        self.first_control=True;self.teacher_mode=options.get('teacher','none')
        self.yaw_integral=0.
        self.leg_action_applied=np.zeros(2,np.float32)
        self.command_time=1.;self.push_time=2.;self.push_end=0.;self.push=np.zeros(3);self.push_torque=np.zeros(3)
        self.position_reference=0.;self.odom_reference=0.;self.travel=0.;self.last_x=self.data.qpos[0]
        if self.native_policy:self.native_policy.reset()
        self._control()
        self.leg_servo=np.asarray(self.out[16:18],np.float32).copy()
        self.leg_hyst=self.leg_servo.copy()
        self.data.ctrl[2:]=np.clip(self.leg_hyst-.063,-.018,.018)
        self.elapsed=0.;self.steps=0;self.errors=[]
        return self._observation(.01),{}

    def step(self,action):
        old_action=self.action.copy();action=np.asarray(action,np.float32)
        # Legacy two-wheel callers keep working with zero leg offsets.
        if action.shape[-1]==2:action=np.concatenate([action,np.zeros(action.shape[:-1]+(2,),np.float32)],-1)
        self.action=np.clip(action,-1,1)
        self._apply_action()
        if self.commands and self.elapsed>=self.command_time:
            forward=self.np_random.uniform(-.16,.16) if self.np_random.random()>.3 else 0.
            yaw=self.np_random.uniform(-.6,.6) if self.np_random.random()>.5 else 0.
            self.command[0]=round(127.5+forward/.21*127.5)
            self.command[1]=round(127.5+yaw/(300*math.pi/180)*127.5)
            self.command[2]=int(self.np_random.uniform(55,200)) if self.domain else 128
            self.command_time=self.elapsed+self.np_random.uniform(*self.command_interval)
        for substep in range(self.policy_ticks):
            while self.delay and self.delay[0][0]<=self.us:self.drive=self.delay.popleft()[1]
            if self.wheel_actuator=='speed':
                # Blend the historical torque model with a finite-bandwidth
                # speed servo. Real drivers accept a speed setpoint but their
                # loop is not ideal; blending reproduces the observed response.
                torque_model=self.torque_limit*(self.drive-self.back_emf*self.data.qvel[self.wheel_v])
                speed_model=self.wheel_eff_inertia*((self.drive/self.back_emf-self.data.qvel[self.wheel_v])/self.speed_tau)
                wanted=np.clip((1.-WHEEL_SPEED_BLEND)*torque_model+WHEEL_SPEED_BLEND*speed_model,
                               -self.torque_limit,self.torque_limit)
            else:
                wanted=np.clip(self.torque_limit*(self.drive-self.back_emf*self.data.qvel[self.wheel_v]),-self.torque_limit,self.torque_limit)
            self.torque+=(wanted-self.torque)*(1-math.exp(-.002/self.motor_tau))
            self.data.ctrl[:2]=self.torque
            self.data.qfrc_applied[self.leg_v]=self.supported_mass*9.81*.5
            if self.pushes and self.elapsed>=self.push_time:
                a=self.np_random.uniform(-math.pi,math.pi);self.push=np.array([math.cos(a),math.sin(a)*.8,0])*self.push_force
                # Body-frame roll/pitch torque, then rotate into the world frame.
                heading=self._angles()[2];tx=self.np_random.uniform(-1,1)*self.push_roll_torque
                ty=self.np_random.uniform(-1,1)*self.push_pitch_torque
                self.push_torque=np.array([tx*math.cos(heading)-ty*math.sin(heading),
                    tx*math.sin(heading)+ty*math.cos(heading),0])
                self.push_end=self.elapsed+.08;self.push_time=self.elapsed+self.np_random.uniform(1.,3.)
            active=self.elapsed<self.push_end
            self.data.xfrc_applied[self.base,:3]=self.push if active else 0
            self.data.xfrc_applied[self.base,3:6]=self.push_torque if active else 0
            mujoco.mj_step(self.model,self.data);self.ticks+=1;self.us+=2000;self.elapsed=self.us/1e6
            if self.ticks%self.feedback_ticks==0:self._read_feedback((self.ticks//self.feedback_ticks-1)%10)
            acceleration=self.data.sensordata[:3];omega=self.data.sensordata[3:6]
            native_acc=np.array([acceleration[1],-acceleration[0],acceleration[2]])
            native_gyro=np.array([-omega[0],-omega[1],omega[2]])*180/math.pi
            if self.vibration_amp:
                native_gyro=native_gyro+self.vibration_amp*math.sin(2*math.pi*self.vibration_hz*self.elapsed)
            if self.domain:
                native_acc+=self.accel_bias+self.np_random.normal(0,.015*self.domain,3)
                native_gyro+=self.gyro_bias+self.np_random.normal(0,.08*self.domain,3)
            if np.dot(native_acc,native_acc)>.4:self.firmware.sense(native_acc,native_gyro,self.us)
            if self.ticks%self.control_ticks==0:
                self._control()
                if substep+1<self.policy_ticks:self._apply_action()
        self.steps+=1;roll,pitch,yaw=self._angles();velocity=self.axle_velocity();yaw_rate=float(self.data.sensordata[5])
        reference=float(self.out[10]);turn=float(self.out[11]);v_error=velocity-reference;yaw_error=yaw_rate-turn
        self.travel+=velocity*self.policy_ticks*.002;self.position_reference+=reference*self.policy_ticks*.002
        yaw_width,yaw_weight=(.08,2.) if self.physics=='measured' else (.15,1.)
        reward=1.+math.exp(-(pitch/.2)**2)+math.exp(-(v_error/.12)**2)+yaw_weight*math.exp(-(yaw_error/yaw_width)**2)
        reward-=.05*float(np.sum(self.action**2))+3.*float(np.sum((self.action-old_action)**2))+.5*roll**2
        # Damp body-rate oscillation explicitly; the leg channels can act on roll.
        rates=self.data.sensordata[3:6]
        reward-=.04*float(rates[0]**2)+.02*float(rates[1]**2)
        position_error=self.travel-self.position_reference
        reward+=1.2*math.exp(-(position_error/.08)**2)
        reward-=.25*min((position_error/.15)**2,100)
        terminated=bool(abs(pitch)>.65 or abs(roll)>.5 or self.data.qpos[2]<.045 or not np.isfinite(self.data.qpos).all())
        if terminated:reward-=50
        truncated=self.elapsed>=self.episode_seconds
        info=dict(velocity=velocity,reference=reference,yaw_rate=yaw_rate,yaw_reference=turn,
            pitch=pitch,roll=roll,time=self.elapsed,position_error=self.travel-self.position_reference,
            wheel_command=self.applied.copy(),fell=terminated)
        return self._observation(self.policy_ticks*.002),float(reward),terminated,truncated,info

    def render(self):
        if self.renderer is None:self.renderer=mujoco.Renderer(self.model,height=720,width=1280)
        camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:]=self.data.xpos[self.base];camera.distance=.48;camera.azimuth=135;camera.elevation=-20
        self.renderer.update_scene(self.data,camera);return self.renderer.render()

    def close(self):
        self.firmware.close()
        if self.renderer is not None:self.renderer.close()
