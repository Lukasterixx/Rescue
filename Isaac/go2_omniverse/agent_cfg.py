# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# WHICH WALKING POLICY THE SIM LOADS.
#
# A FLAT-GROUND POLICY, installed 2026-09-15: `go2_rescue_flat/2026-09-15_08-09-04_abl_FlatP2D`,
# model_1999. The maze is flat now (`terrain_cfg.py`, `elevation_chance=0.0`) because no policy
# trained for this robot climbs its stairs reliably; `training/README.md` has the evidence.
#
# WHAT IT IS. P2Dingo's flat task reproduced inside Rescue's training package
# (`Isaac-Velocity-Rescue-FlatAbl-FlatP2D-v0`, `training/go2_rescue/experiments.py`): P2Dingo's
# reward set on a plane, the bare Go2 with Isaac Lab's stock motor, 2000 iterations, plus the
# +-20 action clip below. It did NOT train with the D1 arm on, and that is deliberate: in a flat
# ablation every policy trained WITH the arm gathered its feet under the body and turned poorly,
# while both trained without it walk the welded robot well.
#
# MEASURED on the robot this sim drives -- arm welded, Unitree's measured motor curve, no mass
# or CoM randomisation, friction 0.8/0.6 -- with `training/measure_bench.py --flat_only`:
#
#                                           this policy    model_7700 (previous)
#   forward tracking @ 0.5 / 1.0 m/s          95 / 97 %         82 / 86 %
#   turn tracking @ 0.2 / 0.5 / 1.0 rad/s     58 / 99 / 99 %    65 / 96 / 99 %
#   drift while turning @ 1.0 rad/s           0.06 m/s          0.06 m/s
#   front / rear foot vs neutral, 0.5 m/s     -2 / -1 cm        -16 / +14 cm
#   same-side front-rear spacing, 1.0 m/s     58 cm             27 cm
#   backward after a 1.0 m/s request          0 %               0 %
#
# The one weak spot is a slow turn: 0.2 rad/s is tracked at 58 %. nav2's DWB never commands
# one (`min_speed_theta` 0.8 rad/s). It trained on the stock motor, not the measured curve this
# sim applies; the numbers above are measured on the measured curve.
#
# THREE THINGS WOULD SILENTLY BREAK THIS DROP-IN. All three hold as of this install.
#   * THE OBSERVATION WIDTH MUST STAY 235. `custom_rl_env.py`'s `ObservationsCfg.PolicyCfg` is
#     a term-for-term copy of the stock `UnitreeGo2RoughEnvCfg`'s, height_scan included
#     (3+3+3+3+12+12+12+187). Under `--arm_mount weld` the robot has 20 joints, and
#     `omniverse_sim._scope_env_cfg_to_legs()` is what narrows the joint terms back to 12.
#   * THE NETWORK SHAPE MUST STAY [512, 256, 128] / elu / no empirical normalisation. The
#     `actor`/`critic` blocks below are what the weights were saved against.
#   * THE ACTION SCALE MUST STAY 0.25, set in `UnitreeGo2CustomEnvCfg.__post_init__`.
#
# THE ROLLBACKS, newest first. All checkpoints are on disk and nothing else needs changing:
#   * The 2026-09-13 stair policy -- climbs a little, gathers its feet, tracks forward at ~82 %.
#     `experiment_name` 'go2_rescue', `load_run` '2026-09-13_12-40-03', `load_checkpoint`
#     'model_7700.pt'. Needs the stairs back only if you want to watch it try them.
#   * P2Dingo's 2026 rough policy. 'go2_p2dingo', '2026-09-12_11-46-13', 'model_1999.pt'.
#   * The original 2024 policy. 'unitree_go2_rough', '.*', 'model_7850_converted.pt'.
unitree_go2_agent_cfg = {
        'seed': 42, 
        'device': 'cuda', 
        'num_steps_per_env': 24, 
        'max_iterations': 15000, 
        'empirical_normalization': False, 
        'obs_groups': {
            'actor': ['policy'],
            'critic': ['policy'],
        },
        'actor': {
            'class_name': 'MLPModel',
            'hidden_dims': [512, 256, 128],
            'activation': 'elu',
        },
        'critic': {
            'class_name': 'MLPModel',
            'hidden_dims': [512, 256, 128],
            'activation': 'elu',
        },
        'algorithm': {
            'class_name': 'PPO', 
            'value_loss_coef': 1.0, 
            'use_clipped_value_loss': True, 
            'clip_param': 0.2, 
            'entropy_coef': 0.01, 
            'num_learning_epochs': 5, 
            'num_mini_batches': 4, 
            'learning_rate': 0.001, 
            'schedule': 'adaptive', 
            'gamma': 0.99, 
            'lam': 0.95, 
            'desired_kl': 0.01, 
            'max_grad_norm': 1.0
        }, 
        'save_interval': 50, 
        # ACTION CLIP, applied by omniverse_sim.py's RslRlVecEnvWrapper. Match the loaded policy's
        # training value: Rescue's go2_rescue / go2_rescue_flat runs from 2026-09-14 on train at
        # 20.0. It never binds on normal output (p99.99 ~8.5, max ~11) and only bounds the
        # last-action feedback runaway, so older unclipped checkpoints walk the same with it.
        'clip_actions': 20.0,
        # The 2026-09-15 flat policy -- see the header at the top of this file.
        # Pinned exactly rather than globbed: `get_checkpoint_path` takes both of these
        # as regexes, so '.*' would start picking whichever run sorts last the moment a
        # second one lands on disk.
        'experiment_name': 'go2_rescue_flat', 
        'load_run': '2026-09-15_08-09-04_abl_FlatP2D', 
        'load_checkpoint': 'model_1999.pt', 
        'run_name': '', 
        'logger': 'tensorboard', 
        'neptune_project': 'isaaclab', 
        'wandb_project': 'isaaclab', 
        'resume': False, 
        }


unitree_g1_agent_cfg = {
        'seed': 42, 
        'device': 'cuda', 
        'num_steps_per_env': 24, 
        'max_iterations': 15000, 
        'empirical_normalization': False, 
        'obs_groups': {
            'actor': ['policy'],
            'critic': ['policy'],
        },
        'actor': {
            'class_name': 'MLPModel',
            'hidden_dims': [512, 256, 128],
            'activation': 'elu',
        },
        'critic': {
            'class_name': 'MLPModel',
            'hidden_dims': [512, 256, 128],
            'activation': 'elu',
        },
        'algorithm': {
            'class_name': 'PPO', 
            'value_loss_coef': 1.0, 
            'use_clipped_value_loss': True, 
            'clip_param': 0.2, 
            'entropy_coef': 0.01, 
            'num_learning_epochs': 5, 
            'num_mini_batches': 4, 
            'learning_rate': 0.001, 
            'schedule': 'adaptive', 
            'gamma': 0.99, 
            'lam': 0.95, 
            'desired_kl': 0.01, 
            'max_grad_norm': 1.0
        }, 
        'save_interval': 50, 
        'experiment_name': 'g1_rough', 
        'run_name': '', 
        'logger': 'tensorboard', 
        'neptune_project': 'isaaclab', 
        'wandb_project': 'isaaclab', 
        'resume': False, 
        'load_run': '.*', 
        'load_checkpoint': 'model_.*_converted.pt'
        }