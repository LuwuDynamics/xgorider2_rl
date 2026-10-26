"""A copy of models/rider_measured.xml whose collision geometry is the real shell: the convex hulls of the CAD meshes of the body (with its
front lip) and of the two legs (with the rear arms), instead of the 90 x 72 x 50 mm box the training model uses.  The box floats 30 mm above the
lowest point of the real body at the front, so it predicts the wrong resting pose after a fall.  Not used for training.

Usage (in a script):  from shell_model import write_shell_xml, patch_loader; path = write_shell_xml(); patch_loader(path)
"""
from pathlib import Path
import mujoco
ROOT = Path(__file__).resolve().parents[1]
_ORIG_FROM_XML = mujoco.MjModel.from_xml_path


def write_shell_xml(path='/tmp/rider_measured_shell.xml', maxhullvert=300):
    text = (ROOT / 'models/rider_measured.xml').read_text()
    meshes = ''.join(f'    <mesh name="col_{n}" file="{ROOT / "models/cad/meshes" / (n + ".stl")}" maxhullvert="{maxhullvert}" />\n' for n in ('base', 'leg12', 'leg22'))
    assert '<asset>' in text
    text = text.replace('<asset>', '<asset>\n' + meshes, 1)
    # the training box is replaced by the hull of the real body; each leg gets the hull of its CAD mesh (links, rear arms)
    text = text.replace('<geom name="chassis" type="box" size=".045 .036 .025" material="shell" />',
                        '<geom name="chassis" type="mesh" mesh="col_base" material="shell" />', 1)
    for leg in ('12', '22'):
        marker = f'<body name="leg{leg}" pos="0 {".0433" if leg == "12" else "-.0433"} -.081">\n        <joint name="extension{leg}" type="slide" axis="0 0 -1" range="-.018 .018" damping="1" armature=".0001" />\n'
        assert marker in text, leg
        text = text.replace(marker, marker + f'        <geom name="hull{leg}" type="mesh" mesh="col_leg{leg}" material="metal" />\n', 1)
    Path(path).write_text(text)
    return path


def patch_loader(shell_path):
    """Make RiderEnv2 load the shell model instead of rider_measured.xml (wraps the ORIGINAL loader once per call)."""
    mujoco.MjModel.from_xml_path = staticmethod(lambda p, *a, **k: _ORIG_FROM_XML(str(shell_path) if 'rider_measured.xml' in str(p) else p, *a, **k))
