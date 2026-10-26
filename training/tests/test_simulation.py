import sys
from pathlib import Path
import unittest
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rider_sim.env import RiderEnv


class SimulationChecks(unittest.TestCase):
    def test_no_chassis_stabilizer_and_zero_action_falls(self):
        env=RiderEnv(commands=False,pushes=False,episode_seconds=8)
        env.reset(seed=123,options={'pitch':.06,'roll':0,'yaw':0})
        while True:
            _,_,done,trunc,info=env.step(np.zeros(2,np.float32))
            self.assertTrue(np.array_equal(env.data.qfrc_applied[:6],np.zeros(6)))
            self.assertTrue(np.array_equal(env.data.xfrc_applied,np.zeros_like(env.data.xfrc_applied)))
            if done or trunc:break
        env.close();self.assertTrue(done)

    def test_native_bus_direction_wrap_and_quantized_speed(self):
        env=RiderEnv(commands=False,pushes=False)
        env.reset(seed=123,options={'pitch':0,'roll':0,'yaw':0})
        for i in range(20):
            env.firmware.feedback(11,(i*100)%1024,100,i*4000,env.feedback)
            env.firmware.feedback(21,(-i*100)%1024,-100,i*4000,env.feedback)
        self.assertGreater(env.feedback[0],75);self.assertGreater(env.feedback[1],75)
        self.assertLess(abs(env.feedback[2]-env.feedback[3]),3)
        self.assertGreater(env.feedback[2],1800)
        env.close()

    def test_repeatable_domain_randomization_and_trajectory(self):
        a=RiderEnv(domain=1);b=RiderEnv(domain=1)
        oa,_=a.reset(seed=10620);ob,_=b.reset(seed=10620)
        np.testing.assert_array_equal(oa,ob)
        for _ in range(20):
            va=a.step([.01,-.01]);vb=b.step([.01,-.01])
            np.testing.assert_array_equal(va[0],vb[0]);self.assertEqual(va[1:4],vb[1:4])
        a.close();b.close()

    def test_display_meshes_keep_physical_parameters(self):
        path=ROOT/'models/cad/rider_cad.xml'
        if not path.exists():self.skipTest('optional CAD display assets not generated')
        a=mujoco.MjModel.from_xml_path(str(ROOT/'models/rider.xml'))
        b=mujoco.MjModel.from_xml_path(str(path))
        for key in ('body_mass','body_inertia','body_ipos','jnt_axis','jnt_pos','dof_armature','actuator_gainprm'):
            np.testing.assert_array_equal(getattr(a,key),getattr(b,key),err_msg=key)


if __name__=='__main__':unittest.main()
