"""Measured mass constraints with explicit, unmeasured CAD distribution priors.

Only the assembly masses are measured. Material proxies, rotor/stator split and
within-part mass centres are uncertain and are sampled during training.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from .mass_properties import combine

ROOT=Path(__file__).resolve().parents[1]
AUDIT=ROOT/'models/mass_audit'
INPUT=AUDIT/'calibrated_distribution.json'
NAMES=('base','cm5','leg12','leg22','wheel11','wheel21')
AXLES=np.array([[0.,.0433,-.063],[0.,-.0433,-.063]])


def fingerprint():
    paths=[INPUT,ROOT/'models/rider_measured.xml',Path(__file__),ROOT/'rider_sim/env.py']
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    return dict(name='measured-20261021',sha256=hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest(),sources_sha256=hashes)


def part(row,mass,center=None):
    return dict(mass_kg=float(mass),center_m=row['center_m'] if center is None else np.asarray(center).tolist(),
        inertia_kg_m2=(np.asarray(row['specific_inertia_m2'])*mass).tolist())


def constrained_masses(rows,total,rng,domain,bounded=False):
    weights=np.array([r['prior_mass_kg'] for r in rows])
    nominal=weights*(total/weights.sum())
    if not domain:return nominal
    for _ in range(256):
        proposal=weights*np.exp(rng.uniform(-.5,.5,len(rows))*domain)
        proposal*=total/proposal.sum()
        if not bounded or np.all((proposal>=nominal*.6)&(proposal<=nominal*1.4)):return proposal
    raise RuntimeError('Could not sample bounded component masses')


def sample(inputs,rng,domain=0.):
    """Keep measured totals; vary allocations and centres within stated priors."""
    d=float(domain);m=inputs['measured_masses_kg'];groups={name:[] for name in NAMES}
    total=m['robot']*(1+rng.uniform(-.025,.025)*d) if d else m['robot']
    cm5_mass=m['cm5']*(1+rng.uniform(-.008,.008)*d) if d else m['cm5']
    battery_mass=m['battery']*(1+rng.uniform(-.01,.01)*d) if d else m['battery']
    servo_masses=np.full(2,m['one_servo'])
    wheel_masses=np.full(2,m['one_wheel'])
    if d:
        servo_masses*=1+rng.uniform(-.02,.02,2)*d
        wheel_masses*=1+rng.uniform(-.02,.02,2)*d
    unknown_mass=total-cm5_mass-battery_mass-servo_masses.sum()-wheel_masses.sum()
    if unknown_mass<=0:raise ValueError('Measured mass constraints have no positive structural remainder')
    for row,mass in zip(inputs['cm5'],constrained_masses(inputs['cm5'],cm5_mass,rng,d,bounded=True)):
        center=np.asarray(row['center_m'])
        if d:center=center+rng.uniform([-.001,-.003,-.003],[.001,.003,.003])*d
        groups['cm5'].append(part(row,mass,center))
    center=np.asarray(inputs['battery']['center_m'])
    if d:center=center+rng.uniform(-.002,.002,3)*d
    groups['base'].append(part(inputs['battery'],battery_mass,center))
    for row,mass in zip(inputs['servos'],servo_masses):groups['base'].append(part(row,mass))
    for row,mass in zip(inputs['structure'],constrained_masses(inputs['structure'],unknown_mass,rng,d)):
        center=np.asarray(row['center_m'])
        if d:center=center+rng.uniform(-.0015,.0015,3)*d
        groups[row['body']].append(part(row,mass,center))
    splits=[]
    for i,(row,mass) in enumerate(zip(inputs['wheels'],wheel_masses)):
        # The complete hub was weighed; its stator/rotor split was not.
        fixed=.55+rng.uniform(-.20,.20)*d if d else .55
        splits.append(float(fixed))
        groups[('leg12','leg22')[i]].append(part(row['stator'],mass*fixed))
        groups[('wheel11','wheel21')[i]].append(part(row['rotor'],mass*(1-fixed)))
    bodies={name:combine(rows) for name,rows in groups.items()}
    if d:
        for body in bodies.values():
            # Uniform central-tensor scaling preserves positive definiteness
            # and the triangle inequality. CG changes already use parallel axes.
            body['inertia_kg_m2']=(np.asarray(body['inertia_kg_m2'])*(1+rng.uniform(-.2,.2)*d)).tolist()
    whole=[]
    for name,body in bodies.items():
        center=np.asarray(body['center_m'])
        if name in ('leg12','wheel11'):center=center+AXLES[0]
        if name in ('leg22','wheel21'):center=center+AXLES[1]
        whole.append(dict(body,center_m=center.tolist()))
    return dict(bodies=bodies,assembly_at_cad_minimum_height=combine(whole),
        total_mass_kg=float(total),stator_mass_fractions=splits,
        fixed_chassis_mass_kg=bodies['base']['mass_kg']+bodies['cm5']['mass_kg'])


class MeasuredCalibration:
    def __init__(self,model):
        self.inputs=json.loads(INPUT.read_text());self.ids={name:model.body(name).id for name in NAMES}
        self.metadata=fingerprint()

    def apply(self,model,rng,domain):
        result=sample(self.inputs,rng,domain)
        for name,row in result['bodies'].items():
            i=self.ids[name];matrix=np.asarray(row['inertia_kg_m2']);values,axes=np.linalg.eigh(matrix)
            if np.linalg.det(axes)<0:axes[:,0]*=-1
            import mujoco
            quat=np.empty(4);mujoco.mju_mat2Quat(quat,axes.ravel())
            model.body_mass[i]=row['mass_kg'];model.body_ipos[i]=row['center_m']
            model.body_inertia[i]=values;model.body_iquat[i]=quat
        return result
