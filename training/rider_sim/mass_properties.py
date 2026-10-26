"""Rigid-body mass moments in SI units, with central tensors and explicit frames."""
import numpy as np


def validate(mass,center,inertia):
    if isinstance(mass,(bool,np.bool_)):raise ValueError('Boolean is not a mass measurement')
    mass=float(mass);center=np.asarray(center,dtype=np.float64);inertia=np.asarray(inertia,dtype=np.float64)
    if not np.isfinite(mass) or mass<=0:raise ValueError('Mass must be a finite positive number in kg')
    if center.shape!=(3,) or not np.isfinite(center).all():raise ValueError('Invalid centre of mass')
    if inertia.shape!=(3,3) or not np.isfinite(inertia).all():raise ValueError('Invalid central inertia tensor')
    if not np.allclose(inertia,inertia.T,atol=1e-14,rtol=1e-10):raise ValueError('Inertia must be symmetric')
    eigenvalues=np.linalg.eigvalsh(inertia)
    if eigenvalues[0]<-1e-14 or eigenvalues[2]>eigenvalues[0]+eigenvalues[1]+1e-12:
        raise ValueError('Central inertia is not physically admissible')
    return mass,center,inertia


def combine(parts):
    """Combine central tensors using the parallel-axis theorem in a common frame."""
    rows=[validate(p['mass_kg'],p['center_m'],p['inertia_kg_m2']) for p in parts]
    if not rows:raise ValueError('No mass properties were supplied')
    total=sum(m for m,_,_ in rows);center=sum(m*c for m,c,_ in rows)/total
    inertia=np.zeros((3,3),np.float64)
    for mass,point,central in rows:
        displacement=point-center
        inertia+=central+mass*(np.dot(displacement,displacement)*np.eye(3)-np.outer(displacement,displacement))
    validate(total,center,inertia)
    return dict(mass_kg=total,center_m=center.tolist(),inertia_kg_m2=inertia.tolist(),
        principal_inertia_kg_m2=np.linalg.eigvalsh(inertia).tolist())


def transform(part,rotation,translation):
    """Express a body's moments in another rigid frame; do not shift central I."""
    mass,center,inertia=validate(part['mass_kg'],part['center_m'],part['inertia_kg_m2'])
    rotation=np.asarray(rotation,np.float64);translation=np.asarray(translation,np.float64)
    if rotation.shape!=(3,3) or not np.allclose(rotation@rotation.T,np.eye(3),atol=1e-10) or not np.isclose(np.linalg.det(rotation),1.):
        raise ValueError('A proper rotation matrix is required')
    if translation.shape!=(3,) or not np.isfinite(translation).all():raise ValueError('Invalid translation')
    return dict(mass_kg=mass,center_m=(rotation@center+translation).tolist(),
        inertia_kg_m2=(rotation@inertia@rotation.T).tolist())
