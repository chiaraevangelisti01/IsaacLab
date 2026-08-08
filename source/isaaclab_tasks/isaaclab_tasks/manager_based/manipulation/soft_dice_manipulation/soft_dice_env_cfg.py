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


def make_deformable_cube_cfg(prim_path: str) -> DeformableObjectCfg:
    return DeformableObjectCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=CUSTOM_DICE_DEFORMABLE_USD,
            scale=CUSTOM_DICE_SCALE,
            physics_material=PhysxDeformableBodyMaterialCfg(
                density=21.5,
                poissons_ratio=0.37,
                youngs_modulus=1.5e4,
                static_friction=1.2,
                dynamic_friction=0.8,
                elasticity_damping=0.02,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.1, 0.1)),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(-0.15, 0.0, 1.01)),
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
                contact_offset=0.008,
                rest_offset=0.002,
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
    # Keep the same absolute position-target interface that passed the parity test.
    # If PPO struggles, the next change should be normalized residual actions.
    joint_pos = base_mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=1.0,
        offset=0.0,
        use_default_offset=False,
        preserve_order=False,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # Current robot state.
        joint_pos = ObsTerm(func=base_mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=base_mdp.joint_vel_rel)

        # Current reference cue.
        reference_joint_pos = ObsTerm(
            func=mdp.reference_joint_pos_rel,
            params={"command_name": "motion", "asset_cfg": SceneEntityCfg("robot")},
        )
        reference_joint_vel = ObsTerm(
            func=mdp.reference_joint_vel,
            params={"command_name": "motion", "asset_cfg": SceneEntityCfg("robot")},
        )

        # Proxy for the previous control command.
        last_action = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_to_reference = EventTerm(
        func=mdp.reset_to_motion_start,
        mode="reset",
        params={
            "command_name": "motion",
            "robot_name": "robot",
            "cube_name": "cube",
            "use_reference_joint_velocity": False,
        },
    )


@configclass
class RewardsCfg:
    # Minimal robot-only tracker first. The cube/object terms come after this learns reliably.
    joint_pos_tracking = RewTerm(
        func=mdp.joint_pos_tracking_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.25,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    joint_vel_tracking = RewTerm(
        func=mdp.joint_vel_tracking_exp,
        weight=0.25,
        params={
            "command_name": "motion",
            "std": 2.0,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    action_rate = RewTerm(
        func=base_mdp.action_rate_l2,
        weight=-0.01,
    )

    joint_limits = RewTerm(
        func=base_mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    # Reaching the end of a reference is a timeout-like natural end, not a failure.
    motion_finished = DoneTerm(
        func=mdp.motion_finished,
        time_out=True,
        params={"command_name": "motion"},
    )

    # Safety fallback only; the motion should normally terminate first.
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)

    # Do NOT use a large-error termination initially: PPO needs to experience bad states
    # while it is learning. We can enable this later if rollouts become pathological.


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
        self.decimation = 1
        self.episode_length_s = 60.0  # hard fallback; motion_finished normally ends earlier

        # Preserve replay parity initially.
        self.sim.dt = 1.0 / 60.0
        self.sim.render_interval = self.decimation
        self.sim.physics = PhysxCfg(
            solver_type=1,
            min_position_iteration_count=32,
            max_position_iteration_count=96,
            min_velocity_iteration_count=4,
            max_velocity_iteration_count=16,
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

        _, fps, root_qpos, object_qpos = load_motion_file(motion_file)

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