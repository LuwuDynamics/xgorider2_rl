"""Model-based linear core used to initialise and to benchmark the learned policy.

``u_c = kp*pitch + kd*pitch_rate_fw + kv*(v_wheel - kappa*v_ref) + kx*position_error``
``u_d = ky*yaw_rate - kref*turn_ref``

wheel commands are ``(u_c + u_d, -u_c + u_d)`` in firmware sign convention. Gains come
from ``tools/design_linear.py`` (robust to +-gain/latency/sensor-scale stress on the
identified plant). It is *not* the deliverable: the PPO stage refines it on the
randomised simulator, and the benchmark tells how much that stage adds.
"""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
KAPPA = 0.28


class LinearPolicy:
    def __init__(self, gains=None, yaw_gain=60., yaw_dc=-1 / .0052, kappa=KAPPA, integral=True):
        if gains is None:
            data = json.loads((ROOT / ('models/robust_linear_gains_i.json' if integral else 'models/robust_linear_gains.json')).read_text())
            gains = data['gains']
        self.kp = float(gains['pitch'])
        self.kd = float(gains['pitch_rate_fw'])
        self.kv = float(gains['wheel_speed'])
        self.kx = float(gains['position'])
        self.ki = float(gains.get('position_integral', 0.))
        self.ky = float(yaw_gain)
        # steady-state turn feed-forward so that yaw_rate -> turn_ref (plant DC gain is negative)
        self.kref = self.ky - yaw_dc
        self.kappa = kappa

    def command(self, raw):
        """raw: unnormalised 18-vector -> (common, differential) command units."""
        v = .5 * (raw[5] + raw[6])
        uc = self.kp * raw[0] + self.kd * raw[2] + self.kv * (v - self.kappa * raw[9]) + self.kx * raw[12] + self.ki * raw[17]
        ud = self.ky * raw[4] - self.kref * raw[10]
        return uc, ud

    def action(self, raw):
        uc, ud = self.command(raw)
        return np.clip(np.array([uc + ud, -uc + ud, 0., 0.]) / 1023., -1, 1).astype(np.float32)

    def as_matrix(self):
        """4 x 18 raw-observation matrix (units/1023) -- used for the supervised initialisation."""
        W = np.zeros((4, 18))
        for j, c in ((0, self.kp), (2, self.kd), (12, self.kx), (17, self.ki)):
            W[0, j] = c
            W[1, j] = -c
        for j in (5, 6):
            W[0, j] = W[1, j] = 0.
            W[0, j] += .5 * self.kv
            W[1, j] -= .5 * self.kv
        W[0, 9] = -self.kv * self.kappa
        W[1, 9] = self.kv * self.kappa
        W[0, 4] += self.ky
        W[1, 4] += self.ky
        W[0, 10] -= self.kref
        W[1, 10] -= self.kref
        return W / 1023.
