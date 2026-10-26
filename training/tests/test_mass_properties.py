"""Independent analytic checks for the CAD mass aggregation, in SI units."""
import unittest
import numpy as np
from rider_sim.mass_properties import combine,transform,validate


def box(mass,center,size):
    x,y,z=size
    return dict(mass_kg=mass,center_m=center,
        inertia_kg_m2=np.diag([mass*(y*y+z*z)/12,mass*(x*x+z*z)/12,mass*(x*x+y*y)/12]))


class MassMomentChecks(unittest.TestCase):
    def test_split_rectangular_solid_equals_whole_solid(self):
        expected=box(.4,[.3,-.2,.1],[.08,.04,.02])
        left=box(.2,[.28,-.2,.1],[.04,.04,.02]);right=box(.2,[.32,-.2,.1],[.04,.04,.02])
        actual=combine([left,right])
        self.assertAlmostEqual(actual['mass_kg'],expected['mass_kg'])
        np.testing.assert_allclose(actual['center_m'],expected['center_m'],atol=1e-14)
        np.testing.assert_allclose(actual['inertia_kg_m2'],expected['inertia_kg_m2'],atol=1e-14)

    def test_translated_pair_of_point_masses(self):
        p=[dict(mass_kg=1.,center_m=[x,2,3],inertia_kg_m2=np.zeros((3,3))) for x in (-.1,.1)]
        actual=combine(p)
        np.testing.assert_allclose(actual['center_m'],[0,2,3],atol=1e-14)
        np.testing.assert_allclose(actual['inertia_kg_m2'],np.diag([0,.02,.02]),atol=1e-14)

    def test_coordinate_rotation_preserves_principal_moments(self):
        part=box(.05,[.02,.01,.03],[.08,.04,.02])
        r=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        actual=transform(part,r,[.01,.02,.03])
        np.testing.assert_allclose(actual['center_m'],[0,.04,.06],atol=1e-14)
        np.testing.assert_allclose(np.linalg.eigvalsh(actual['inertia_kg_m2']),np.linalg.eigvalsh(part['inertia_kg_m2']))

    def test_unphysical_or_unknown_mass_is_rejected(self):
        for mass in (0,-1,float('nan')):
            with self.assertRaises(ValueError):validate(mass,[0,0,0],np.eye(3))
        with self.assertRaises(ValueError):validate(1,[0,0,0],np.diag([1,1,3]))
        with self.assertRaises(ValueError):combine([])


if __name__=='__main__':unittest.main()
