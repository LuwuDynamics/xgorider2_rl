"""What would "quantising" the v6 policy do?  A demonstration with the real weights (artifacts/balance-v6/actor.json).

Quantisation here is *fake*: every weight is rounded to the grid of a smaller number format and converted back to float32, so the unchanged
float32 C++ policy (the same code that runs on the ESP32) can run it in the simulator.  That shows what rounding does to the control; it says
nothing about the speed of a real integer kernel.

Usage: .venv/bin/python tools/quant_demo.py [--bits] [--outputs] [--closed-loop]      (no argument = all three, about 4 minutes)

--bits         the bit pattern of one real weight in float32 / float16 / bfloat16 / int8 / int4, and statistics of the weights
--outputs      error of the network output (in wheel-command units, 0..1023) on 10 000 observations recorded from simulated driving and pushing
--closed-loop  the robot balancing and driving in the simulator (randomised plant, random commands and pushes, 30 s) with each variant
Results 2026-10-23: runs/quant_demo.log.
"""
import sys, os, json, re, struct, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from multiprocessing import Pool
HDR = ROOT / 'artifacts/balance-v6/policy_weights.h'
ACTOR = json.load(open(ROOT / 'artifacts/balance-v6/actor.json'))
W0 = [np.array(l['weight'], np.float32) for l in ACTOR['layers']]
B0 = [np.array(l['bias'], np.float32) for l in ACTOR['layers']]
MEAN = np.array(ACTOR['observation_mean'], np.float32)
INV_STD = np.array(ACTOR['observation_inv_std'], np.float32)


# ------------------------------------------------------------------------------------------------ number formats
def bits32(x):
    u = struct.unpack('>I', struct.pack('>f', x))[0]
    return f'{u >> 31:01b} {(u >> 23) & 255:08b} {u & 0x7FFFFF:023b}'


def to_fp16(w):
    return w.astype(np.float16).astype(np.float32)


def to_bf16(w):
    """bfloat16: float32 with the lower 16 bits dropped (round to nearest even)."""
    u = np.ascontiguousarray(w, np.float32).view(np.uint32).astype(np.uint64)
    u = (u + 0x7FFF + ((u >> 16) & 1)) & 0xFFFF0000
    return u.astype(np.uint32).view(np.float32)


def to_int(w, bits, per_row):
    """Symmetric integer grid: w ~ q * scale, q an integer in [-(2^(bits-1)-1), 2^(bits-1)-1]; one scale per tensor or per output row."""
    top = 2 ** (bits - 1) - 1
    if per_row and w.ndim == 2:
        scale = np.maximum(np.abs(w).max(axis=1, keepdims=True), 1e-12) / top
    else:
        scale = max(float(np.abs(w).max()), 1e-12) / top
    q = np.clip(np.round(w / scale), -top, top)
    return (q * scale).astype(np.float32), q.astype(np.int32), scale


VARIANTS = {   # name -> function(w) applied to every weight matrix and bias vector
    'float32 (as is)': lambda w: w,
    'float16': to_fp16,
    'bfloat16': to_bf16,
    'int8 (one scale per layer)': lambda w: to_int(w, 8, False)[0],
    'int8 (one scale per row)': lambda w: to_int(w, 8, True)[0],
    'int6 (per row)': lambda w: to_int(w, 6, True)[0],
    'int5 (per row)': lambda w: to_int(w, 5, True)[0],
    'int4 (per row)': lambda w: to_int(w, 4, True)[0],
    'int3 (per row)': lambda w: to_int(w, 3, True)[0],
    'int2 (per row)': lambda w: to_int(w, 2, True)[0],
}


def forward(raw, W, B, act_bits=0):
    """The network exactly as policy.cpp: normalise, clip +-5, tanh, tanh, linear, clip +-1.  act_bits > 0 also rounds the activations."""
    def q(a, top):
        if not act_bits:
            return a
        levels = 2 ** (act_bits - 1) - 1
        return np.round(np.clip(a, -top, top) / top * levels) / levels * top
    x = q(np.clip((raw - MEAN) * INV_STD, -5, 5).astype(np.float32), 5.0)
    a = q(np.tanh(x @ W[0].T + B[0]), 1.0)
    b = q(np.tanh(a @ W[1].T + B[1]), 1.0)
    return np.clip(b @ W[2].T + B[2], -1, 1)


# ------------------------------------------------------------------------------------------------ part 1: what the numbers look like
def show_bits():
    w = W0[0]
    x = float(w[0, 0])
    print(f'one real weight (layer 1, row 0, column 0): {x!r}')
    print(f'  float32 : {bits32(np.float32(x))}   = 1 sign bit, 8 exponent bits, 23 fraction bits -> {float(np.float32(x))!r}')
    h = np.float16(x)
    print(f'  float16 : {int(np.array(h).view(np.uint16)):016b}   -> {float(h)!r}  (error {abs(float(h) - x):.1e})')
    b = float(to_bf16(np.array([x], np.float32))[0])
    print(f'  bfloat16: {int(np.array([x], np.float32).view(np.uint32)[0] >> 16):016b} (cut)   -> {b!r}  (error {abs(b - x):.1e})')
    for bits in (8, 4):
        deq, q, scale = to_int(w, bits, True)
        print(f'  int{bits}    : integer {int(q[0, 0])} with this row\'s scale {float(scale[0, 0]):.6f}   -> {float(deq[0, 0])!r}  (error {abs(float(deq[0, 0]) - x):.1e})')
    print()
    allw = np.concatenate([a.ravel() for a in W0 + B0])
    nz = np.count_nonzero(allw)
    print(f'the whole network: {allw.size} numbers, {nz} not exactly zero (the zero ones are the inputs the exporter switches off and the two unused leg outputs)')
    print(f'  size of the numbers: smallest nonzero |w| {np.abs(allw[allw != 0]).min():.2e}, largest {np.abs(allw).max():.2f}, median |w| {np.median(np.abs(allw[allw != 0])):.3f}')
    for i, (w_, b_) in enumerate(zip(W0, B0)):
        print(f'  layer {i + 1}: weights {w_.shape}, largest |w| {np.abs(w_).max():.2f}, typical |w| {np.median(np.abs(w_[w_ != 0])):.3f}; bias largest {np.abs(b_).max():.2f}')
    print('  bytes: float32 %d, float16 %d, int8 %d (+ scales), int4 %d (+ scales)' % (allw.size * 4, allw.size * 2, allw.size, allw.size // 2))
    print()


# ------------------------------------------------------------------------------------------------ part 2: error of the output
def record_observations(n_episodes=10, seconds=20):
    from rider_sim.env2 import RiderEnv2
    from rider_sim.native_policy import NativePolicy
    native = NativePolicy(str(HDR))
    obs, ref = [], []
    for seed in range(n_episodes):
        env = RiderEnv2(domain=1.0, commands=True, pushes=True, episode_seconds=seconds, native_policy=native, noise=1.0)
        env.reset(seed=seed)
        while True:
            _, _, term, trunc, _ = env.step(native.action.copy())
            obs.append(native.observation.copy())
            ref.append(native.action.copy())
            if term or trunc:
                break
        env.close()
    native.close()
    return np.array(obs, np.float32), np.array(ref, np.float32)


def show_outputs():
    print('recording observations from simulated driving and pushing ...', flush=True)
    obs, cpp = record_observations()
    ref = forward(obs, W0, B0)
    print(f'{len(obs)} observations; my numpy copy of the network vs the C++ policy: largest difference {np.abs(ref - cpp).max() * 1023:.5f} wheel-command units (float32 rounding order only)')
    print('output error in wheel-command units (the wheel command runs -1023..1023; the robot\'s wheel dead zone alone is 12-45 units, the sensor noise much more)')
    print(f'{"variant":30s} {"mean |error|":>13s} {"99th pct":>10s} {"largest":>9s}')
    for name, f in VARIANTS.items():
        W = [f(w) for w in W0]
        B = [f(b) for b in B0]
        err = np.abs(forward(obs, W, B)[:, :2] - ref[:, :2]) * 1023
        print(f'{name:30s} {err.mean():13.3f} {np.percentile(err, 99):10.2f} {err.max():9.2f}')
    for bits in (8, 6):
        err = np.abs(forward(obs, [to_int(w, 8, True)[0] for w in W0], [to_int(b, 8, True)[0] for b in B0], act_bits=bits)[:, :2] - ref[:, :2]) * 1023
        print(f'{"int8 weights + int%d activations" % bits:30s} {err.mean():13.3f} {np.percentile(err, 99):10.2f} {err.max():9.2f}')
    print()


# ------------------------------------------------------------------------------------------------ part 3: the robot with the rounded weights
def fmt(x):
    s = '%.9g' % x
    return s + 'f' if ('.' in s or 'e' in s) else s + '.0f'


def write_header(path, W, B):
    text = HDR.read_text()
    for name, arr in zip(('w1', 'b1', 'w2', 'b2', 'w3', 'b3'), (W[0], B[0], W[1], B[1], W[2], B[2])):
        flat = arr.ravel()
        body = ''.join('        ' + ','.join(fmt(v) for v in flat[i:i + 8]) + ',\n' for i in range(0, len(flat), 8))
        text, n = re.subn(r'(static const float %s\[\d+\]=\{\n).*?(    \};)' % name, lambda m: m.group(1) + body + m.group(2), text, count=1, flags=re.S)
        assert n == 1, name
    Path(path).write_text(text)


def closed_loop(a):
    name, header, seed = a
    os.chdir(ROOT)
    from rider_sim.env2 import RiderEnv2
    from rider_sim.native_policy import NativePolicy
    native = NativePolicy(header)
    env = RiderEnv2(domain=1.0, commands=True, pushes=True, episode_seconds=30, native_policy=native, noise=1.0)
    env.reset(seed=seed)
    pitch, tv, verr = [], 0., []
    while True:
        _, _, term, trunc, info = env.step(native.action.copy())
        pitch.append(float(np.degrees(info['pitch'])))
        tv += float(info['wheel_tv'])
        verr.append(float(info['velocity'] - info['reference']))
        if term or trunc:
            break
    t = env.elapsed
    env.close(); native.close()
    return dict(name=name, seed=seed, fell=bool(term), t=t, pitch_sd=float(np.std(pitch)), tv_per_s=tv / max(t, 1e-6) / 1023., v_err=float(np.sqrt(np.mean(np.square(verr)))))


def show_closed_loop(seeds=16):
    out = Path('/tmp/quant_headers')
    out.mkdir(exist_ok=True)
    jobs = []
    for k, (name, f) in enumerate(VARIANTS.items()):
        path = out / f'v{k}.h'
        write_header(path, [f(w) for w in W0], [f(b) for b in B0])
        jobs += [(name, str(path), s) for s in range(seeds)]
    began = time.monotonic()
    with Pool(9) as p:
        rows = p.map(closed_loop, jobs, chunksize=1)
    print(f'closed loop in the simulator: randomised plant, random drive commands and pushes, 30 s, {seeds} seeds per variant (same seeds for all)')
    print(f'{"variant":30s} {"fell":>6s} {"pitch sd [deg]":>15s} {"wheel jitter":>13s} {"speed error [m/s]":>18s}')
    for name in VARIANTS:
        rr = [r for r in rows if r['name'] == name]
        print(f'{name:30s} {sum(r["fell"] for r in rr):3d}/{len(rr):<2d} {np.mean([r["pitch_sd"] for r in rr]):15.2f} {np.mean([r["tv_per_s"] for r in rr]):13.2f} {np.mean([r["v_err"] for r in rr]):18.3f}')
    print(f'(wall {time.monotonic() - began:.0f} s; "wheel jitter" = total variation of the wheel command per second, in units of full scale)')
    print()


if __name__ == '__main__':
    todo = [a for a in sys.argv[1:] if a.startswith('--')] or ['--bits', '--outputs', '--closed-loop']
    if '--bits' in todo:
        show_bits()
    if '--outputs' in todo:
        show_outputs()
    if '--closed-loop' in todo:
        show_closed_loop()
