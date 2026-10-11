# PPO runner for the Rescue Go2 task (copied from P2Dingo's, 2026-09-12).
#
# THE NETWORK SHAPE IS NOT A FREE CHOICE. `go2_omniverse/agent_cfg.py` loads the checkpoint
# with `hidden_dims [512, 256, 128]`, `activation "elu"` and `empirical_normalization
# False` for both actor and critic. A policy trained at any other width, depth or
# normalisation setting will not load into the sim -- it fails on a state-dict shape
# mismatch, which is a confusing way to discover a tuning decision. These values match the
# stock `UnitreeGo2RoughPPORunnerCfg` exactly and should be changed only alongside
# `agent_cfg.py`.
#
# Note `UnitreeGo2FlatPPORunnerCfg` narrows the net to [128, 128, 128]. Do NOT inherit from
# it just because this task runs on flat ground -- that width is incompatible with the sim.

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import (RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg,
                                RslRlSymmetryCfg)

from ..krail_symmetry import compute_symmetric_states


@configclass
class Go2RescuePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    # 4000, NOT P2DINGO'S 2000, BECAUSE THIS RUN HAS A TERRAIN LADDER TO CLIMB. The 2026
    # rough policy's run stopped at 2000 with `Curriculum/terrain_levels` at 1.5 of 10 and
    # still rising; the promotion rule has since been re-ruled (mdp/curriculums.py) so the
    # ladder is climbable, but a level is earned one episode at a time -- 20 s, i.e. ~42
    # iterations at 24 steps per env -- and the deployment riser (10 cm) sits at level ~4.5
    # of the stair terrains in terrain_cfg.py. Budget: ~625 iterations for the command
    # curriculum to reach full range, a few hundred more for the terrain ladder, and the
    # rest at full difficulty. `save_interval` means an earlier checkpoint is available if
    # the curve has flattened; stop the run rather than wait it out.
    max_iterations = 8000
    save_interval = 50
    # Its own experiment directory, so this never writes into `logs/rsl_rl/unitree_go2_rough`
    # beside `model_7850_converted.pt` -- the checkpoint the sim currently depends on.
    experiment_name = "go2_rescue"
    # ACTIONS ARE CLIPPED (2026-09-14). Without it every run of this task ended in the same
    # blow-up; `Go2RescueExpPPORunnerCfg` below has the mechanism and the measurements. +-20
    # never touches normal output (p99.99 is ~8.5). The sim must clip at the same value.
    clip_actions = 20.0
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_obs_normalization=False,
        critic_obs_normalization=False,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        # 0.01, AND IT IS LOAD-BEARING. DO NOT LOWER IT WITHOUT READING THIS.
        #
        # Spot's own config uses 0.0025 and it is tempting to match it, since this task runs
        # Spot's rewards. That was tried (run 2026-09-11_21-18-28) on the reasoning that the
        # policy was being paid to stay noisy and taking it: `Policy/mean_std` fell to 0.60
        # by iteration 250, climbed back to 0.71 and sat there while mean reward had been
        # flat since 1500. The change did exactly what it was supposed to -- std ended at
        # 0.16 instead of 0.71, every regularisation penalty roughly halved, yaw tracking
        # error fell 0.63 -> 0.21 -- and it destroyed the gait:
        #
        #                     stride    duty    foot lift    step length
        #   0.01   walk 0.5   2.0 Hz    51 %      10.1 cm        20.9 cm
        #   0.0025 walk 0.5   7.1 Hz    70 %       1.4 cm         4.5 cm
        #   0.0025 turn 1.0   8.3 Hz    68 %       1.6 cm         1.9 cm
        #
        # It stopped walking and started skating -- buzzing its feet a centimetre off the
        # ground and scuffing through turns without lifting at all.
        #
        # WHY, and this is the part that matters: the shuffle is the better-scoring policy
        # under this reward set and always was. Every regularisation penalty is cheaper with
        # small fast motions -- action_smoothness -4.02 -> -2.05, joint_pos -0.74 -> -0.19,
        # base_motion -0.85 -> -0.33 and so on, about 3.7 reward units gained -- against
        # roughly 1.1 lost on `air_time` and `gait`. Nothing prices swing height, because
        # `foot_clearance` weights its height error by the foot's HORIZONTAL SPEED and so
        # scores a foot that never moves as perfect (measured: 0.4234 for the trot, 0.4192
        # for the shuffle -- the term cannot see the difference).
        #
        # So the entropy bonus is not buying exploration here, it is acting as a GUARD: a
        # 1.4 cm shuffle is a delicate thing to execute and cannot be done while being kicked
        # by +-0.71 of action noise. That is an accident rather than a design, and the honest
        # fix is to make the shuffle unprofitable -- give `foot_clearance` a contact-based
        # swing gate instead of a velocity-based one, so a planted foot stops scoring. Until
        # someone does that, this number is the only thing standing between the reward set
        # and a skating robot.
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class Go2RescueFlatPPORunnerCfg(Go2RescuePPORunnerCfg):
    """Same everything, its own log directory.

    The network shape is inherited deliberately — see the note above; it is pinned by
    `go2_omniverse/agent_cfg.py` and must not narrow just because this variant runs on flat
    ground. (The stock `UnitreeGo2FlatPPORunnerCfg` drops to [128, 128, 128], which is the
    trap this avoids.)

    Only `experiment_name` differs, so the quick flat checks do not pile up in the same
    directory as the rough runs. `agent_cfg.py` resolves a checkpoint with
    `load_run: ".*"`, and two kinds of run sharing a directory is how the wrong policy ends
    up on the robot.
    """

    def __post_init__(self):
        super().__post_init__()
        self.experiment_name = "go2_rescue_flat"


@configclass
class Go2RescueExpPPORunnerCfg(Go2RescuePPORunnerCfg):
    """The short fine-tune experiments' runner (../experiments.py): the same, with actions clipped.

    WHY THE CLIP. Every run of this task has blown up the same way: value loss to 1e13-1e30 and
    then "normal expects all elements of std >= 0.0". The control fine-tune from model_7650 did
    it again within 155 iterations. The mechanism is a feedback loop, not bad physics: actions
    are unclipped, the previous action is an observation, and `action_smoothness` is
    ||a_t - a_{t-1}||. One unusually large output feeds its own next input, the next output is
    larger, the joint targets saturate at the torque limits (so the physics and every other
    reward term stay bounded, which is exactly what run 3's logs showed), and
    `action_smoothness` alone reaches ~1e7 per step, which the value function cannot absorb.

    It is a property of the policy, not of training noise: measure_bench.py's deterministic
    rollouts of model_7650 saw |a| reach 28 682 on one robot, where the p99.99 of normal output
    is 8.0-8.5 and the largest in a clean run 11.0. Clipping at 20 never touches normal output
    and bounds the loop: the fed-back observation, the joint target and the penalty with it.

    The clip now lives on `Go2RescuePPORunnerCfg` itself, so this class adds nothing; it is kept
    so the experiment task ids still resolve.
    """


@configclass
class Go2KRailPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """The K-Rail Square policy (`krail_env_cfg.py`). Stock rough-Go2 PPO, in rsl-rl 5's actor/critic form, the critic
    reading the uncorrupted observation group."""

    num_steps_per_env = 24
    max_iterations = 6000
    save_interval = 100
    experiment_name = "go2_krail"
    # Raw actions clipped at +-20, as this package's other runs (the README's last-action feedback runaway). Isaac
    # Lab's action-term `clip` would clip the joint TARGETS and leave the raw action, which is what feeds back, free.
    # The exported policy clamps the same (`export_krail.py`), so the sim's last_action matches training's.
    clip_actions = 20.0
    obs_groups = {"actor": ["policy"], "critic": ["critic"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[512, 256, 128], activation="elu", obs_normalization=False)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
        # Every minibatch trained on as recorded and mirrored left-right (`krail_symmetry.py`): v2 turned well one
        # way and badly the other, and swapped sides between checkpoints. On from v2's iteration 2000.
        # The mirror loss joined at 2400: with augmentation alone the policy's mean stayed lopsided (rsl-rl's
        # symmetry metric ~1.0, not falling) and its action noise rose 0.87 -> 1.10 in 400 iterations, the mirrored
        # actions being unlikely under a lopsided mean. The loss pulls the mean symmetric directly.
        symmetry_cfg=RslRlSymmetryCfg(use_data_augmentation=True, use_mirror_loss=True, mirror_loss_coeff=0.5,
                                      data_augmentation_func=compute_symmetric_states),
    )
