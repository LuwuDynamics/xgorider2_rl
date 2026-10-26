"""Contract-6 training pipeline.

Stages (each is logged and reproducible):

1. **Plant**: ``rider_sim/plant.py`` samples wheel plants that reproduce the *measured* real
   frequency response (``models/measured_frf.json``); ``env2.py`` runs the firmware C++ in the loop.
2. **Initialisation**: a robust linear controller (``tools/design_linear.py``, stress-tested for
   +-gain, extra latency, mis-scaled wheel sensor) is cloned into the 18-32-32-4 tanh actor.
   This is *not* the old continuous-time LQR: it is designed on the delayed, identified plant.
3. **PPO**: critic warm-up with the actor frozen, then PPO on the randomised plant family with a
   reward that penalises command rate/jerk and body rate.  Checkpoints are ranked by an
   acceptance score that contains survival *and* jitter (``evaluate2.py``), and the BC init is
   evaluated first, so PPO is only accepted if it actually improves on it.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from multiprocessing import Pool
from pathlib import Path
import shutil
import time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv, VecNormalize

from rider_sim.env2 import RiderEnv2, OBS_MEAN, OBS_STD2, MASKED_INPUTS
from rider_sim.linear_policy import LinearPolicy

ROOT = Path(__file__).resolve().parent
EVAL_ACTIVE = None   # set in main(): the scenarios the checkpoint ranking uses
EVAL_SCENARIOS = ('stand_push', 'drive', 'stress_delay', 'stress_weak', 'corner', 'hard_push')
EVAL_FRICTION = ('stand_fric', 'stand_fric_hi')


def reward_config(args):
    return dict(rate=args.rate_penalty, jerk=args.jerk_penalty, body_rate=args.body_rate_penalty,
                still_v=args.still_v, still_x=args.still_x)


def make_factory(index, args, counts):
    def factory():
        nominal = index in counts['nominal']
        env = RiderEnv2(domain=0. if nominal else args.domain, commands=index >= counts['stand'], pushes=True,
                        episode_seconds=args.episode_seconds, seed=args.seed + index, noise=1.0,
                        command_interval=(4., 8.) if index % 2 == 0 else (1.5, 3.),
                        reward=reward_config(args), corner_fraction=0. if nominal else args.corner_fraction,
                        friction_prob=args.friction_prob, friction_units=(args.friction_lo, args.friction_hi))
        return env
    return factory


# ----------------------------------------------------------------------------- BC data
def _collect(job):
    seed, steps, noise = job
    teacher = LinearPolicy()
    rng = np.random.default_rng(seed)
    env = RiderEnv2(domain=1., commands=True, pushes=True, episode_seconds=12, seed=seed, noise=1.0,
                    command_interval=(1.5, 4.))
    obs, _ = env.reset(seed=seed)
    X, Y = [], []
    while len(X) < steps:
        target = teacher.action(env.raw_observation)
        X.append(obs.copy())
        Y.append(target)
        action = target + rng.normal(0, noise, 4).astype(np.float32) * np.array([1, 1, 0, 0], np.float32)
        obs, _, term, trunc, _ = env.step(action)
        if term or trunc:
            obs, _ = env.reset()
    env.close()
    return np.array(X, np.float32), np.array(Y, np.float32)


def collect_bc(steps, workers, seed):
    jobs = [(seed + 17 * i, steps // workers, noise) for i, noise in enumerate(np.linspace(0.01, 0.08, workers))]
    with Pool(workers) as pool:
        out = pool.map(_collect, jobs)
    return np.concatenate([o[0] for o in out]), np.concatenate([o[1] for o in out])


def behaviour_clone(model, X, Y, epochs, lr=1e-3, batch=2048, seed=0):
    torch.manual_seed(seed)
    actor = torch.nn.Sequential(model.policy.mlp_extractor.policy_net, model.policy.action_net)
    optimizer = torch.optim.Adam(actor.parameters(), lr=lr)
    x = torch.from_numpy(X)
    y = torch.from_numpy(Y)
    n = len(x)
    steps = epochs * (n // batch)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, steps)
    last = 0.
    for step in range(steps):
        idx = torch.randint(0, n, (batch,))
        loss = torch.mean((actor(x[idx]) - y[idx]) ** 2)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        scheduler.step()
        last = float(loss)
        if step % 1000 == 0:
            print(f'bc step {step}/{steps} mse {last:.6f}', flush=True)
    with torch.no_grad():
        err = (actor(x[:20000]) - y[:20000]).abs()
    return dict(final_mse=last, max_abs_action_error=float(err.max()), mean_abs_action_error=float(err.mean()))


# ----------------------------------------------------------------------------- callbacks
class CriticWarmup(BaseCallback):
    """Freeze the actor until the critic has seen ``steps`` of on-policy data."""

    def __init__(self, steps):
        super().__init__()
        self.steps = steps
        self.frozen = False
        self.start = 0

    def _actor_params(self):
        p = self.model.policy
        return list(p.mlp_extractor.policy_net.parameters()) + list(p.action_net.parameters()) + [p.log_std]

    def _on_training_start(self):
        self.start = self.model.num_timesteps      # a resumed model keeps counting: the warm-up is measured from here
        if self.steps > 0:
            for param in self._actor_params():
                param.requires_grad_(False)
            self.frozen = True

    def _on_step(self):
        if self.frozen and self.num_timesteps - self.start >= self.steps:
            for param in self._actor_params():
                param.requires_grad_(True)
            self.frozen = False
            print(f'critic warm-up finished at {self.num_timesteps}; actor unfrozen', flush=True)
        return True


def acceptance_score(rows):
    survived = np.mean([r['survived'] for r in rows])
    fric = [r for r in rows if r['scenario'].startswith('stand_fric')]
    base = [r for r in rows if not r['scenario'].startswith('stand_fric')]
    ok = [r for r in base if r['survived'] and 'cmd_hf_rms' in r]
    if not ok:
        return -1e3
    med = lambda k: float(np.median([r[k] for r in ok]))
    score = float(100 * survived - 0.8 * med('cmd_hf_rms') - 0.3 * med('rate_std_dps') - 60 * med('vel_mae') - 10 * med('pos_final'))
    # standing on wheels with a dead zone: mean ground speed (0.003 = still, 0.03 = the stepping cycle)
    fric_ok = [r for r in fric if r['survived'] and 'vel_mae' in r]
    if fric:
        score -= 1000 * (float(np.mean([r['vel_mae'] for r in fric_ok])) if fric_ok else 0.1)
    return score


def evaluate_model(model, folder, tag, seeds, seconds=15, workers=8):
    import evaluate2
    path = folder / f'eval-{tag}.zip'
    model.save(path)
    jobs = [(('ppo', str(path)), s, 71000 + i, seconds, 1) for s in EVAL_ACTIVE for i in range(seeds)]
    with Pool(workers) as pool:
        rows = pool.map(evaluate2.run_episode, jobs, chunksize=1)
    path.unlink(missing_ok=True)
    return acceptance_score(rows), evaluate2.summarize(rows)


class Evaluation(BaseCallback):
    def __init__(self, folder, every, seeds, workers):
        super().__init__()
        self.folder = folder
        self.every = every
        self.seeds = seeds
        self.workers = workers
        self.next = every
        self.best = -1e9
        self.began = time.monotonic()

    def evaluate(self, tag):
        score, summary = evaluate_model(self.model, self.folder, tag, self.seeds, workers=self.workers)
        record = dict(tag=tag, timesteps=self.num_timesteps, seconds=time.monotonic() - self.began, score=score, summary=summary)
        with (self.folder / 'evaluations.jsonl').open('a') as stream:
            stream.write(json.dumps(record) + '\n')
        print('EVALUATION', tag, 'score', round(score, 2),
              {k: f"{v['survived']}/{v['n']}" for k, v in summary.items()}, flush=True)
        self.model.save(self.folder / f'step-{tag}')
        if score > self.best:
            self.best = score
            self.model.save(self.folder / 'best')
            print('SAVED BEST', round(score, 2), flush=True)
        return score

    def _on_training_start(self):
        self.evaluate('init')

    def _on_step(self):
        if self.num_timesteps >= self.next:
            self.next = self.num_timesteps + self.every
            self.evaluate(str(self.num_timesteps))
        return True


def sources_digest():
    paths = [ROOT / 'train2.py', ROOT / 'evaluate2.py', ROOT / 'models/rider_measured.xml', ROOT / 'models/measured_frf.json',
             ROOT / 'models/robust_linear_gains_i.json'] + list((ROOT / 'rider_sim').glob('*.py')) + list((ROOT / 'native').glob('*.cpp'))
    firmware = ROOT.parent / 'firmware/src'
    paths += list((firmware / 'policy').glob('policy.*')) + list(firmware.glob('*.cpp')) + list((firmware / 'control').glob('*'))
    return {str(p.relative_to(ROOT.parent)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT / 'runs/v2-a')
    parser.add_argument('--steps', type=int, default=6_000_000)
    parser.add_argument('--envs', type=int, default=8)
    parser.add_argument('--seed', type=int, default=4242)
    parser.add_argument('--domain', type=float, default=1.0)
    parser.add_argument('--episode-seconds', type=float, default=20.)
    parser.add_argument('--stand-fraction', type=float, default=.2)
    parser.add_argument('--nominal-fraction', type=float, default=.125)
    parser.add_argument('--learning-rate', type=float, default=4e-5)
    parser.add_argument('--sigma', type=float, default=.03, help='initial exploration std (action units)')
    parser.add_argument('--warmup-steps', type=int, default=300_000)
    parser.add_argument('--bc-steps', type=int, default=240_000)
    parser.add_argument('--bc-epochs', type=int, default=30)
    parser.add_argument('--eval-every', type=int, default=500_000)
    parser.add_argument('--eval-seeds', type=int, default=6)
    parser.add_argument('--eval-workers', type=int, default=8)
    parser.add_argument('--rate-penalty', type=float, default=6.0)
    parser.add_argument('--jerk-penalty', type=float, default=3.0)
    parser.add_argument('--body-rate-penalty', type=float, default=.02)
    parser.add_argument('--corner-fraction', type=float, default=0.4)
    parser.add_argument('--still-v', type=float, default=0., help='reward weight: speed while standing (no command)')
    parser.add_argument('--still-x', type=float, default=0., help='reward weight: position error while standing')
    parser.add_argument('--friction-prob', type=float, default=0., help='share of episodes with a wheel dead zone / friction')
    parser.add_argument('--friction-lo', type=float, default=10.)
    parser.add_argument('--friction-hi', type=float, default=55.)
    parser.add_argument('--n-steps', type=int, default=512)
    parser.add_argument('--batch-size', type=int, default=2048)
    parser.add_argument('--ent-coef', type=float, default=0.)
    parser.add_argument('--from-scratch', action='store_true', help='skip the linear-controller initialisation')
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--reset-sigma', type=float, default=0., help='when resuming: set the wheel exploration std (action units) to this')
    parser.add_argument('--resume-warmup', action='store_true', help='also warm the critic up (actor frozen) when resuming: use it when the reward changed')
    parser.add_argument('--dummy', action='store_true', help='single-process envs (debugging)')
    args = parser.parse_args()
    torch.set_num_threads(1)
    global EVAL_ACTIVE
    EVAL_ACTIVE = EVAL_SCENARIOS + (EVAL_FRICTION if args.friction_prob > 0 else ())
    args.output.mkdir(parents=True, exist_ok=True)
    counts = dict(stand=int(args.envs * args.stand_fraction),
                  nominal=set(np.linspace(0, args.envs, int(args.envs * args.nominal_fraction), endpoint=False).astype(int)))
    factories = [make_factory(i, args, counts) for i in range(args.envs)]
    venv = (DummyVecEnv if args.dummy else (lambda f: SubprocVecEnv(f, start_method='fork')))(factories)
    env = VecNormalize(venv, norm_obs=False, norm_reward=True, clip_reward=10, gamma=.997)
    config = dict(started_at_utc=datetime.now(timezone.utc).isoformat(), args={k: str(v) for k, v in vars(args).items()},
                  observation_contract=6, architecture=[18, 32, 32, 4], masked_inputs=list(MASKED_INPUTS),
                  observation_mean=OBS_MEAN.tolist(), observation_std=OBS_STD2.tolist(), sources_sha256=sources_digest())
    if args.resume:
        model = PPO.load(args.resume, env=env, device='cpu', learning_rate=args.learning_rate)
        config['resume'] = str(args.resume)
        if args.reset_sigma > 0:
            with torch.no_grad():
                model.policy.log_std[:2] = math.log(args.reset_sigma)
            config['reset_sigma'] = args.reset_sigma
    else:
        model = PPO('MlpPolicy', env, policy_kwargs=dict(net_arch=dict(pi=[32, 32], vf=[64, 64]), activation_fn=torch.nn.Tanh,
                                                         log_std_init=math.log(args.sigma)),
                    learning_rate=args.learning_rate, n_steps=args.n_steps, batch_size=args.batch_size, n_epochs=5,
                    gamma=.997, gae_lambda=.95, clip_range=.1, ent_coef=args.ent_coef, target_kl=.015,
                    max_grad_norm=.5, seed=args.seed, device='cpu', verbose=1)
        with torch.no_grad():
            model.policy.log_std[2:] = math.log(1e-3)
        if not args.from_scratch:
            began = time.monotonic()
            X, Y = collect_bc(args.bc_steps, min(10, args.envs + 2), args.seed)
            print('bc samples', len(X), 'collect seconds', round(time.monotonic() - began, 1), flush=True)
            config['behaviour_cloning'] = behaviour_clone(model, X, Y, args.bc_epochs, seed=args.seed)
            config['behaviour_cloning']['samples'] = int(len(X))
            config['init'] = 'robust linear controller (models/robust_linear_gains_i.json) cloned into the actor'
            print('bc result', config['behaviour_cloning'], flush=True)
        else:
            config['init'] = 'random'
        # the legs are pinned to zero in this contract-6 iteration: keep their parameters (and noise) fixed
        mask = torch.tensor([1., 1., 0., 0.])
        model.policy.action_net.weight.register_hook(lambda g: g * mask[:, None])
        model.policy.action_net.bias.register_hook(lambda g: g * mask)
        model.policy.log_std.register_hook(lambda g: g * mask)
        with torch.no_grad():
            model.policy.action_net.weight[2:] = 0.
            model.policy.action_net.bias[2:] = 0.
    (args.output / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
    model.save(args.output / 'init')
    model.rider_contract = 6
    callbacks = [Evaluation(args.output, args.eval_every, args.eval_seeds, args.eval_workers)]
    if args.warmup_steps and (not args.resume or args.resume_warmup):
        callbacks.insert(0, CriticWarmup(args.warmup_steps))
    model.learn(total_timesteps=args.steps, callback=callbacks, reset_num_timesteps=not bool(args.resume))
    model.save(args.output / 'last')
    env.save(str(args.output / 'vec_normalize.pkl'))
    env.close()


if __name__ == '__main__':
    main()
