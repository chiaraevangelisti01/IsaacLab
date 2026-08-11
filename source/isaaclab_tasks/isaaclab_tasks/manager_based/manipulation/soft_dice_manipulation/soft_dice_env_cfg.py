from __future__ import annotations

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
import isaaclab.envs.mdp as base_mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass
from isaaclab.utils.noise import UniformNoiseCfg as Unoise
from isaaclab_assets.robots.unitree_h1_aist import H1_FIXED_CFG
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim import PhysxDeformableBodyMaterialCfg
from . import mdp
from .mdp.motion_utils import (
    CUSTOM_DICE_DEFORMABLE_USD,
    CUSTOM_DICE_SCALE,
    desired_cube_pose_from_holosoma,
    load_motion_file,
    validate_asset_paths,
)


DEFAULT_CUBE_SIZE = 0.31
DEFAULT_TABLE_LENGTH = 0.80
DEFAULT_TABLE_WIDTH = 1.20
DEFAULT_GROUND_Z = 0.0

H1_TRACKING_JOINT_NAMES = [
    "torso",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
]

H1_TRACKING_ACTION_SCALE = mdp.build_joint_action_scale(
    robot_cfg=H1_FIXED_CFG,
    joint_names=H1_TRACKING_JOINT_NAMES,
    scale_factor=0.25,
)


def make_deformable_cube_cfg(
    prim_path: str,
) -> DeformableObjectCfg:
    return DeformableObjectCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=CUSTOM_DICE_DEFORMABLE_USD,
            scale=CUSTOM_DICE_SCALE,

            collision_props=sim_utils.CollisionPropertiesCfg(
                rest_offset=0.001,
                contact_offset=0.005,
            ),

            physics_material=PhysxDeformableBodyMaterialCfg(
                density=21.5,
                poissons_ratio=0.37,
                youngs_modulus=1.5e4,
                static_friction=1.2,
                dynamic_friction=0.8,
                elasticity_damping=0.02,
            ),

            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.8, 0.1, 0.1)
            ),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(
            pos=(-0.15, 0.0, 1.01)
        ),
        debug_vis=False,
    )


@configclass
class SoftDiceSceneCfg(InteractiveSceneCfg):
    plane = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0.40],
            rot=[0.0, 0.0, 0.0, 1.0],
        ),
        spawn=sim_utils.CuboidCfg(
            size=(DEFAULT_TABLE_LENGTH, DEFAULT_TABLE_WIDTH, 0.80),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.004,
                rest_offset=0.001,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.35, 0.35)),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.1,
                dynamic_friction=0.03,
                restitution=0.0,
                friction_combine_mode="average",
                restitution_combine_mode="average",
            ),
        ),
    )

    cube: DeformableObjectCfg = make_deformable_cube_cfg("{ENV_REGEX_NS}/Cube")

    robot: ArticulationCfg = H1_FIXED_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            rot=(0.0, 0.0, 1.0, 0.0),
            pos=(0.0, 0.0, 1.06),
        ),
    )


@configclass
class CommandsCfg:
    motion = mdp.MotionCommandCfg(
        asset_name="robot",
        motion_file="",
        start_frame=0,
        playback_speed=1.0,
        loop=False,
    )


@configclass
class ActionsCfg:
    joint_pos = base_mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=H1_TRACKING_JOINT_NAMES,
        scale=H1_TRACKING_ACTION_SCALE,
        use_default_offset=True,
        preserve_order=True,
    )


@configclass
class ObservationsCfg:
    """BeyondMimic-style observations adapted to fixed-base H1."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Actor observations."""

        # ----------------------------------------------------------
        # Reference command: q_ref + qdot_ref.
        # BeyondMimic provides these directly, not relative to
        # default joint positions.
        # ----------------------------------------------------------
        command = ObsTerm(
            func=mdp.reference_joint_command,
            params={
                "command_name": "motion",
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=H1_TRACKING_JOINT_NAMES,
                    preserve_order=True,
                ),
            },
        )

        # ----------------------------------------------------------
        # Demonstrated torso orientation relative to current torso.
        # ----------------------------------------------------------
        motion_anchor_ori_b = ObsTerm(
            func=mdp.motion_anchor_ori_b,
            params={
                "command_name": "motion",
            },
            noise=Unoise(
                n_min=-0.05,
                n_max=0.05,
            ),
        )

        # ----------------------------------------------------------
        # Current controlled joint state.
        # ----------------------------------------------------------
        joint_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=H1_TRACKING_JOINT_NAMES,
                    preserve_order=True,
                ),
            },
            noise=Unoise(
                n_min=-0.01,
                n_max=0.01,
            ),
        )

        joint_vel = ObsTerm(
            func=base_mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=H1_TRACKING_JOINT_NAMES,
                    preserve_order=True,
                ),
            },
            noise=Unoise(
                n_min=-0.5,
                n_max=0.5,
            ),
        )

        actions = ObsTerm(
            func=base_mdp.last_action
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class PrivilegedCfg(ObsGroup):
        """Critic observations."""

        command = ObsTerm(
            func=mdp.reference_joint_command,
            params={
                "command_name": "motion",
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=H1_TRACKING_JOINT_NAMES,
                    preserve_order=True,
                ),
            },
        )

        motion_anchor_ori_b = ObsTerm(
            func=mdp.motion_anchor_ori_b,
            params={
                "command_name": "motion",
            },
        )

        # Actual Cartesian body state relative to torso.
        body_pos = ObsTerm(
            func=mdp.robot_body_pos_b,
            params={
                "command_name": "motion",
            },
        )

        body_ori = ObsTerm(
            func=mdp.robot_body_ori_b,
            params={
                "command_name": "motion",
            },
        )

        joint_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=H1_TRACKING_JOINT_NAMES,
                    preserve_order=True,
                ),
            },
        )

        joint_vel = ObsTerm(
            func=base_mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=H1_TRACKING_JOINT_NAMES,
                    preserve_order=True,
                ),
            },
        )

        actions = ObsTerm(
            func=base_mdp.last_action
        )

    policy: PolicyCfg = PolicyCfg()
    critic: PrivilegedCfg = PrivilegedCfg()

    
@configclass
class EventCfg:
    """BeyondMimic-style randomization adapted to fixed-base H1."""

    # --------------------------------------------------------------
    # Startup domain randomization.
    # --------------------------------------------------------------

    physics_material = EventTerm(
        func=base_mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                body_names=".*",
            ),
            "static_friction_range": (0.3, 1.6),
            "dynamic_friction_range": (0.3, 1.2),
            "restitution_range": (0.0, 0.5),
            "num_buckets": 64,
        },
    )

    add_joint_default_pos = EventTerm(
        func=mdp.randomize_joint_default_pos,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=H1_TRACKING_JOINT_NAMES,
                preserve_order=True,
            ),
            "pos_distribution_params": (-0.01, 0.01),
            "operation": "add",
        },
    )

    base_com = EventTerm(
        func=base_mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                body_names="torso_link",
            ),
            "com_range": {
                "x": (-0.025, 0.025),
                "y": (-0.05, 0.05),
                "z": (-0.05, 0.05),
            },
        },
    )

    # --------------------------------------------------------------
    # Episode reset.
    # --------------------------------------------------------------

    reset_to_reference = EventTerm(
        func=mdp.reset_to_motion_start,
        mode="reset",
        params={
            "command_name": "motion",
            "robot_name": "robot",
            "cube_name": "cube",

            # BeyondMimic initializes qdot from the motion.
            "use_reference_joint_velocity": True,

            # BeyondMimic uses ±0.1 rad.
            "joint_position_range": (-0.1, 0.1),

            "tracking_asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=H1_TRACKING_JOINT_NAMES,
                preserve_order=True,
            ),
        },
    )

    filter_lower_body_self_collisions = EventTerm(
        func=mdp.filter_lower_body_self_collisions,
        mode="prestartup",
        params={
            "lower_body_names": [
                "left_hip_yaw_link",
                "left_hip_roll_link",
                "left_hip_pitch_link",
                "left_knee_link",
                "left_ankle_link",
                "right_hip_yaw_link",
                "right_hip_roll_link",
                "right_hip_pitch_link",
                "right_knee_link",
                "right_ankle_link",
            ],
        },
    )


@configclass
class RewardsCfg:
    """BeyondMimic-style robot motion-tracking rewards."""

    # --------------------------------------------------------------
    # Anchor tracking.
    # --------------------------------------------------------------
    motion_global_anchor_ori = RewTerm(
        func=mdp.motion_global_anchor_orientation_error_exp,
        weight=0.5,
        params={
            "command_name": "motion",
            "std": 0.4,
        },
    )

    # --------------------------------------------------------------
    # Cartesian body pose tracking.
    # --------------------------------------------------------------
    motion_body_pos = RewTerm(
        func=mdp.motion_relative_body_position_error_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.3,
        },
    )

    motion_body_ori = RewTerm(
        func=mdp.motion_relative_body_orientation_error_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.4,
        },
    )

    # --------------------------------------------------------------
    # Cartesian body velocity tracking.
    # --------------------------------------------------------------
    motion_body_lin_vel = RewTerm(
        func=mdp.motion_global_body_linear_velocity_error_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 1.0,
        },
    )

    motion_body_ang_vel = RewTerm(
        func=mdp.motion_global_body_angular_velocity_error_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 3.14,
        },
    )

    # --------------------------------------------------------------
    # Regularization.
    # --------------------------------------------------------------
    action_rate = RewTerm(
        func=base_mdp.action_rate_l2,
        weight=-0.1,
    )

    joint_limits = RewTerm(
        func=base_mdp.joint_pos_limits,
        weight=-10.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )


@configclass
class TerminationsCfg:
    """Tracking terminations adapted from BeyondMimic."""

    # --------------------------------------------------------------
    # Task-specific temporary ending.
    #
    # Keep this while we always start from frame 0.
    # Remove it later if/when we implement trajectory-phase resampling.
    # --------------------------------------------------------------
    motion_finished = DoneTerm(
        func=mdp.motion_finished,
        time_out=True,
        params={
            "command_name": "motion",
        },
    )

    # Hard environment fallback.
    time_out = DoneTerm(
        func=base_mdp.time_out,
        time_out=True,
    )

    # --------------------------------------------------------------
    # BeyondMimic-style tracking failure.
    #
    # Their terminal bodies are wrists/ankles. Our distal tracked
    # upper-body references are the two elbows--> TO DO PUT VIRTUAL HANDS 
    # --------------------------------------------------------------
    ee_body_pos = DoneTerm(
        func=mdp.bad_motion_body_pos_z_only,
        params={
            "command_name": "motion",
            "threshold": 0.25,
            "body_names": [
                "left_elbow_link",
                "right_elbow_link",
            ],
        },
    )
@configclass
class SoftDiceTrackingEnvCfg(ManagerBasedRLEnvCfg):
    scene: SoftDiceSceneCfg = SoftDiceSceneCfg(num_envs=256, env_spacing=2.0, replicate_physics=False)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    # Parameters needed to reconstruct the same table layout as the validated replay.
    cube_size: float = DEFAULT_CUBE_SIZE
    table_length: float = DEFAULT_TABLE_LENGTH
    table_width: float = DEFAULT_TABLE_WIDTH
    ground_z: float = DEFAULT_GROUND_Z

    def __post_init__(self):
        self.decimation = 4  # 50 Hz control for H1 tracking, 200 Hz sim
        self.episode_length_s = 60.0  # hard fallback; motion_finished normally ends earlier

        self.sim.dt = 1.0 / 200.0
        self.sim.render_interval = self.decimation
        self.sim.physics = PhysxCfg(
            solver_type=1,
            min_position_iteration_count=8,
            max_position_iteration_count=64,
            min_velocity_iteration_count=1,
            max_velocity_iteration_count=4,
            bounce_threshold_velocity=0.2,
        )

        self.viewer.eye = (-2.6, 0.0, 1.5)
        self.viewer.lookat = (0.0, 0.0, 1.1)

    def configure_from_motion(self) -> None:
        """Resolve scene layout from the selected trajectory before scene creation."""
        validate_asset_paths()

        motion_file = self.commands.motion.motion_file
        if not motion_file:
            raise ValueError(
                "No motion file configured. For training pass e.g. "
                "env.commands.motion.motion_file=/absolute/path/to/demo.npz"
            )

        _, _, fps, root_qpos, object_qpos, _,_,_, = load_motion_file(motion_file)

        if root_qpos is None or object_qpos is None:
            raise ValueError(
                "Soft-dice tracking currently expects root + H1 + object qpos (33 columns)."
            )

        start_frame = int(self.commands.motion.start_frame)
        if not 0 <= start_frame < root_qpos.shape[0]:
            raise ValueError(f"start_frame={start_frame} outside the motion range.")

        robot_pos = np.asarray(self.scene.robot.init_state.pos, dtype=np.float32)
        robot_quat = np.asarray(self.scene.robot.init_state.rot, dtype=np.float32)

        cube_pos, _, *_ = desired_cube_pose_from_holosoma(
            root_qpos,
            object_qpos,
            start_frame,
            robot_pos,
            robot_quat,
        )

        self.scene.cube.init_state.pos = cube_pos.tolist()

        cube_height = float(self.cube_size) * float(CUSTOM_DICE_SCALE[2])
        cube_bottom_z = float(cube_pos[2] - cube_height / 2.0)
        table_thickness = cube_bottom_z - float(self.ground_z)
        if table_thickness <= 0.0:
            raise ValueError(f"Computed table thickness is not positive: {table_thickness}")

        table_pos = np.asarray(self.scene.table.init_state.pos, dtype=np.float32)
        table_pos[0] = cube_pos[0] - 0.25
        table_pos[1] = cube_pos[1]
        table_pos[2] = float(self.ground_z) + table_thickness / 2.0

        self.scene.table.init_state.pos = table_pos.tolist()
        self.scene.table.spawn.size = (
            float(self.table_length),
            float(self.table_width),
            float(table_thickness),
        )

        # Keep the hard timeout just beyond the trajectory duration.
        remaining_frames = root_qpos.shape[0] - start_frame
        duration = remaining_frames / (float(fps) * float(self.commands.motion.playback_speed))
        step_dt = float(self.decimation) * float(self.sim.dt)
        self.episode_length_s = max(duration + step_dt, step_dt)

    def validate_config(self):
        # Isaac Lab 3 beta2 invokes this after Hydra overrides are applied.
        # This is why env.commands.motion.motion_file=... can also drive the dependent scene layout.
        self.configure_from_motion()