import json
from pathlib import Path
import sys
import unittest
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rider_sim.calibration import INPUT,AXLES,sample
from rider_sim.env import RiderEnv
from rider_sim.mass_properties import validate


class MeasuredModelChecks(unittest.TestCase):
    @unittest.skipUnless((ROOT/'models/cad/rider_measured_cad.xml').exists(), 'the CAD display files (models/cad/, non-commercial license) are optional')
    def test_cad_display_preserves_calibrated_dynamics(self):
        a=mujoco.MjModel.from_xml_path(str(ROOT/'models/rider_measured.xml'))
        b=mujoco.MjModel.from_xml_path(str(ROOT/'models/cad/rider_measured_cad.xml'))
        for key in ('body_mass','body_inertia','body_ipos','body_iquat','jnt_axis','jnt_pos','dof_armature','actuator_gainprm'):
            np.testing.assert_array_equal(getattr(a,key),getattr(b,key),err_msg=key)

    def test_mass_constraints_and_cm5_location(self):
        inputs=json.loads(INPUT.read_text());r=sample(inputs,np.random.default_rng(1))
        self.assertAlmostEqual(sum(b['mass_kg'] for b in r['bodies'].values()),.5783,places=12)
        self.assertAlmostEqual(r['bodies']['cm5']['mass_kg'],.0628,places=12)
        self.assertTrue(.033<r['bodies']['cm5']['center_m'][0]<.040)
        for side in (0,1):
            leg=r['bodies'][('leg12','leg22')[side]];wheel=r['bodies'][('wheel11','wheel21')[side]]
            self.assertGreater(leg['mass_kg'],.1329*.55)
            self.assertAlmostEqual(wheel['mass_kg'],.1329*.45)

    def test_compiled_tensor_and_full_assembly_moments(self):
        # The calibration maths is checked without the empirical CoM offset the old env applies on top.
        import rider_sim.env as old_env
        saved,old_env.COM_X_SHIFT=old_env.COM_X_SHIFT,0.
        env=RiderEnv(physics='measured',commands=False,pushes=False)
        try:
            env.reset(seed=900,options={'pitch':0,'roll':0,'yaw':0})
            report=env.physics_sample
            for name,row in report['bodies'].items():
                idx=env.model.body(name).id;rot=np.empty(9);mujoco.mju_quat2Mat(rot,env.model.body_iquat[idx]);rot=rot.reshape(3,3)
                tensor=rot@np.diag(env.model.body_inertia[idx])@rot.T
                np.testing.assert_allclose(tensor,row['inertia_kg_m2'],rtol=1e-11,atol=1e-14)
            com=np.asarray(report['assembly_at_cad_minimum_height']['center_m'])+env.data.xpos[env.base]
            np.testing.assert_allclose(env.data.subtree_com[env.base],com,atol=1e-12)
            self.assertAlmostEqual(env.supported_mass,env.model.body_mass[env.base]+env.model.body_mass[env.model.body('cm5').id])
        finally:
            env.close();old_env.COM_X_SHIFT=saved

    def test_random_allocations_are_physical_and_preserve_constraints(self):
        inputs=json.loads(INPUT.read_text());rng=np.random.default_rng(42)
        for _ in range(100):
            r=sample(inputs,rng,1.)
            self.assertAlmostEqual(sum(b['mass_kg'] for b in r['bodies'].values()),r['total_mass_kg'],places=12)
            self.assertTrue(.5783*.975<=r['total_mass_kg']<=.5783*1.025)
            for b in r['bodies'].values():validate(b['mass_kg'],b['center_m'],b['inertia_kg_m2'])

    def test_replay_and_no_hidden_balance_force(self):
        env=RiderEnv(physics='measured',domain=1,commands=False,pushes=False,episode_seconds=2)
        try:
            def run():
                env.reset(seed=123,options={'pitch':.06,'roll':0,'yaw':0});trace=[]
                while True:
                    obs,_,done,trunc,info=env.step([0,0]);trace.append(obs)
                    np.testing.assert_array_equal(env.data.qfrc_applied[:6],np.zeros(6))
                    np.testing.assert_array_equal(env.data.xfrc_applied,np.zeros_like(env.data.xfrc_applied))
                    if done or trunc:return np.asarray(trace),done
            a,done=run();b,_=run();self.assertTrue(done);np.testing.assert_array_equal(a,b)
        finally:env.close()


if __name__=='__main__':unittest.main()
