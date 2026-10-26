"""Plots for the documentation, and for your own runs.

    python tools/plot_results.py all                                  # the four data plots + the pipeline diagram, for the shipped model
    python tools/plot_results.py frf      [--output docs/images/frf.png]
    python tools/plot_results.py margin   [--output docs/images/loop_margin.png]
    python tools/plot_results.py curve runs/v6/evaluations.jsonl [--average-from 1500000 --average-to 3500000] [--output ...]
    python tools/plot_results.py survival artifacts/balance-v6/evaluation_native.json [--output ...]
    python tools/plot_results.py pipeline [--output docs/images/pipeline.png]

frf       the real robot's wheel-command -> pitch-rate frequency response (models/measured_frf.json) and how well the calibrated simulator reproduces it
margin    loop gain and sensitivity of the previous policy and of v6 on that REAL response (artifacts/balance-v6/loop_margin_vs_v5.txt, from tools/loop_margin.py)
curve     acceptance score of every PPO checkpoint (runs/<name>/evaluations.jsonl written by train2.py)
survival  survivors per scenario of an evaluate2.py report
pipeline  the whole pipeline as one picture
"""
import argparse
import json
from pathlib import Path
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

ROOT = Path(__file__).resolve().parents[1]
IMAGES = ROOT / 'docs/images'
BLUE, ORANGE, GREEN, RED, GRAY, INK = '#2563eb', '#f59e0b', '#16a34a', '#dc2626', '#94a3b8', '#0f172a'
plt.rcParams.update({'font.size': 11, 'axes.edgecolor': INK, 'axes.labelcolor': INK, 'xtick.color': INK, 'ytick.color': INK,
                     'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': True, 'grid.color': '#e2e8f0', 'figure.dpi': 140,
                     'savefig.bbox': 'tight', 'savefig.facecolor': 'white'})


def save(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
    print('wrote', path.relative_to(ROOT) if path.is_relative_to(ROOT) else path)


def frf(output):
    data = json.loads((ROOT / 'models/measured_frf.json').read_text())
    f = data['freqs_hz']
    rows = {}
    check = ROOT / 'artifacts/balance-v6/sim_frf_check.txt'
    if check.exists():
        for line in check.read_text().splitlines():
            m = re.match(r'\s*([\d.]+)\s*\|\s*([\d.]+)\s+(-?[\d.]+)\s*\|\s*([\d.]+)\s+(-?[\d.]+)', line)
            if m:
                rows[float(m.group(1))] = (float(m.group(2)), float(m.group(3)))
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.6))
    a.plot(f, data['rate_mag'], 'o-', color=BLUE, label='real robot (median of %d balance chunks)' % data['chunks'])
    if rows:
        sim_f = sorted(rows)
        sim_mag = [m * r[0] for m, r in zip(data['rate_mag'], [rows[x] for x in sim_f])]
        a.plot(sim_f, sim_mag, 's--', color=ORANGE, label='calibrated simulator')
    a.set_xlabel('frequency [Hz]'); a.set_ylabel('pitch rate per wheel-command unit [rad/s]'); a.set_title('magnitude'); a.legend(frameon=False, fontsize=9)
    b.plot(f, data['rate_phase_deg'], 'o-', color=BLUE, label='real robot')
    if rows:
        sim_phase = [p + r[1] for p, r in zip(data['rate_phase_deg'], [rows[x] for x in sim_f])]
        b.plot(sim_f, sim_phase, 's--', color=ORANGE, label='calibrated simulator')
        worst = max(abs(rows[x][1]) for x in rows)
        b.text(0.03, 0.05, f'worst phase difference {worst:.1f}°', transform=b.transAxes, fontsize=9)
    b.set_xlabel('frequency [Hz]'); b.set_ylabel('phase [deg]'); b.set_title('phase (an integrator is -90°; the rest is delay and lag)'); b.legend(frameon=False, fontsize=9)
    fig.suptitle('The wheel plant: identified from the real robot, reproduced by the simulator', y=1.02)
    save(fig, output)


def parse_margin(path):
    policies, current = {}, None
    for line in path.read_text().splitlines():
        m = re.match(r'== (policy|reference): (\S+)', line)
        if m:
            current = {'name': m.group(2), 'f': [], 'phi': [], 'one_minus': []}
            policies[m.group(1)] = current
            continue
        m = re.match(r'\s+([\d.]+)\s+([\d.]+)\s+(-?\d+)\s+([\d.]+)\s*$', line)
        if m and current is not None:
            current['f'].append(float(m.group(1))); current['phi'].append(float(m.group(2))); current['one_minus'].append(float(m.group(4)))
        m = re.search(r'sensitivity peak on measured plant: ([\d.]+)\s+max \|Phi\| above 8 Hz: ([\d.]+)', line)
        if m and current is not None:
            current['peak'] = float(m.group(1)); current['above8'] = float(m.group(2))
    return policies


def margin(output):
    p = parse_margin(ROOT / 'artifacts/balance-v6/loop_margin_vs_v5.txt')
    new, old = p['policy'], p['reference']
    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.6))
    a.plot(old['f'], old['phi'], 'o-', color=RED, label=f"previous policy (v5), above 8 Hz max {old['above8']:.2f}")
    a.plot(new['f'], new['phi'], 'o-', color=GREEN, label=f"v6, above 8 Hz max {new['above8']:.2f}")
    a.axhline(1.0, color=INK, lw=0.8, ls=':')
    a.set_xlabel('frequency [Hz]'); a.set_ylabel('|loop gain Φ|'); a.set_title('loop gain on the measured plant'); a.legend(frameon=False, fontsize=9)
    b.plot(old['f'], [1 / x for x in old['one_minus']], 'o-', color=RED, label=f"v5: sensitivity peak {old['peak']:.1f}")
    b.plot(new['f'], [1 / x for x in new['one_minus']], 'o-', color=GREEN, label=f"v6: sensitivity peak {new['peak']:.2f}")
    b.set_yscale('log'); b.set_xlabel('frequency [Hz]'); b.set_ylabel('sensitivity 1/|1 - Φ|'); b.set_title('how much a disturbance is amplified'); b.legend(frameon=False, fontsize=9)
    fig.suptitle('Why the previous policy rang at 7-14 Hz on the robot and v6 does not', y=1.02)
    save(fig, output)


def curve(path, output, average_from=None, average_to=None):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    steps = [r['timesteps'] for r in rows]
    scores = [r['score'] for r in rows]
    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    ax.plot(steps, scores, 'o-', color=BLUE)
    if average_from is not None and average_to is not None:
        ax.axvspan(average_from, average_to, color=GREEN, alpha=0.15, label=f'averaged for export ({average_from / 1e6:.1f}-{average_to / 1e6:.1f} M steps)')
        ax.legend(frameon=False)
    ax.set_xlabel('PPO steps'); ax.set_ylabel('acceptance score'); ax.set_title('Acceptance score of every checkpoint (survival + jitter + tracking)')
    ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f'{v / 1e6:g} M'))
    save(fig, output)


def survival(path, output):
    summary = json.loads(Path(path).read_text())['summary']
    names = list(summary)
    frac = [summary[n]['survived'] / summary[n]['n'] for n in names]
    fig, ax = plt.subplots(figsize=(8.5, 3.8))
    bars = ax.barh(names[::-1], [100 * x for x in frac[::-1]], color=[GREEN if x == 1 else ORANGE if x > 0.85 else RED for x in frac[::-1]])
    for bar, n in zip(bars, names[::-1]):
        ax.text(bar.get_width() + 1, bar.get_y() + bar.get_height() / 2, f"{summary[n]['survived']}/{summary[n]['n']}", va='center', fontsize=9)
    ax.set_xlim(0, 112); ax.set_xlabel('survived [%] (native C++ controller, 20 s episodes)'); ax.set_title('Simulation acceptance suite')
    ax.grid(axis='y', visible=False)
    save(fig, output)


def pipeline(output):
    steps = [
        ('real robot trace', '55 Hz: command, pitch rate,\nwheel speed', '#e0f2fe'),
        ('system identification', 'Welch cross-spectra →\nfrequency response', '#e0f2fe'),
        ('plant family', 'torque source + delay + lag,\nrefitted to that response', '#fef3c7'),
        ('MuJoCo + firmware', 'IMU filter, legs, servo bus,\npolicy controller (C++ in the loop)', '#fef3c7'),
        ('robust linear teacher', 'delay-aware, stress-tested\ngains', '#dcfce7'),
        ('behaviour cloning', 'teacher → 18-32-32-4\ntanh network', '#dcfce7'),
        ('PPO', 'critic warm-up, corner\nsampling, jitter penalties', '#dcfce7'),
        ('checkpoint average', '1.5-3.5 M steps,\nacceptance score', '#dcfce7'),
        ('export + verify', 'float32 C++ header,\nnative == reference', '#fce7f3'),
        ('acceptance + margins', 'survival / jitter suite,\nmargins on the REAL response', '#fce7f3'),
        ('firmware', 'host tests → build → flash,\ntilt / lift / free-fall guards', '#ede9fe'),
    ]
    cols = 4
    fig, ax = plt.subplots(figsize=(11, 5.2))
    ax.set_xlim(0, cols * 2.9); ax.set_ylim(0, 3 * 1.9); ax.axis('off'); ax.grid(False)
    pos = []
    for i, (title, body, color) in enumerate(steps):
        row, col = divmod(i, cols)
        if row % 2 == 1:
            col = cols - 1 - col
        x, y = col * 2.9 + 0.1, (2 - row) * 1.9 + 0.2
        ax.add_patch(FancyBboxPatch((x, y), 2.5, 1.35, boxstyle='round,pad=0.02,rounding_size=0.12', fc=color, ec=INK, lw=1))
        ax.text(x + 1.25, y + 1.05, f'{i + 1}. {title}', ha='center', va='center', fontsize=9.4, fontweight='bold', color=INK)
        ax.text(x + 1.25, y + 0.5, body, ha='center', va='center', fontsize=8.8, color=INK)
        pos.append((x, y))
    for (x0, y0), (x1, y1) in zip(pos, pos[1:]):
        if abs(y0 - y1) < 0.1:
            start, end = ((x0 + 2.5, y0 + 0.67), (x1, y1 + 0.67)) if x1 > x0 else ((x0, y0 + 0.67), (x1 + 2.5, y1 + 0.67))
        else:
            start, end = (x0 + 1.25, y0), (x1 + 1.25, y1 + 1.35)
        ax.add_patch(FancyArrowPatch(start, end, arrowstyle='-|>', mutation_scale=14, color=INK, lw=1.2))
    ax.set_title('Rider2-RL: from a real robot trace to a policy on the ESP32', fontsize=13, color=INK)
    save(fig, output)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('what', choices=['all', 'frf', 'margin', 'curve', 'survival', 'pipeline'])
    parser.add_argument('input', nargs='?', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--average-from', type=int)
    parser.add_argument('--average-to', type=int)
    args = parser.parse_args()
    out = lambda name: args.output or IMAGES / name
    if args.what == 'all':
        frf(IMAGES / 'frf.png')
        margin(IMAGES / 'loop_margin.png')
        curve(ROOT / 'artifacts/balance-v6/ppo_evaluations.jsonl', IMAGES / 'training_curve.png', 1_500_000, 3_500_000)
        survival(ROOT / 'artifacts/balance-v6/evaluation_native.json', IMAGES / 'survival.png')
        pipeline(IMAGES / 'pipeline.png')
    elif args.what == 'frf':
        frf(out('frf.png'))
    elif args.what == 'margin':
        margin(out('loop_margin.png'))
    elif args.what == 'curve':
        curve(args.input or ROOT / 'artifacts/balance-v6/ppo_evaluations.jsonl', out('training_curve.png'), args.average_from, args.average_to)
    elif args.what == 'survival':
        survival(args.input or ROOT / 'artifacts/balance-v6/evaluation_native.json', out('survival.png'))
    else:
        pipeline(out('pipeline.png'))


if __name__ == '__main__':
    main()
