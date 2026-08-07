from __future__ import annotations

from dataclasses import MISSING

import numpy as np

import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as base_mdp
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass
from isaaclab_assets.robots.unitree_h1_aist import H1_FIXED_CFG
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim import PhysxDeformableBodyMaterialCfg

from scripts.soft_dice_environment.replay_utils import (
    CUSTOM_DICE_DEFORMABLE_USD,
    CUSTOM_DICE_SCALE,
    desired_cube_pose_from_holosoma,
    load_motion_file,
)

from . import mdp


DEFAULT_CUBE_SIZE = 0.31
DEFAULT_TABLE_LENGTH = 0.80
DEFAULT_TABLE_WIDTH = 1.20
DEFAULT_GROUND_Z = 0.0


def make_deformable_cube_cfg(prim_path: str) -> DeformableObjectCfg:
    """Same deformable dice configuration used by the working replay."""
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
        debug_vis=True,
    )


@configclass
class SoftDiceSceneCfg(InteractiveSceneCfg):
    plane = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(
            intensity=3000.0,
            color=(0.75, 0.75, 0.75),
        ),
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
        motion_file=MISSING,
        start_frame=0,
        playback_speed=1.0,
        loop=False,
    )


@configclass
class ActionsCfg:
    # Absolute H1 joint position targets. 
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
        joint_pos = ObsTerm(func=base_mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=base_mdp.joint_vel_rel)
        reference = ObsTerm(func=base_mdp.generated_commands, params={"command_name": "motion"})
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
    # Placeholder reward for the parity test. 
    alive = RewTerm(func=base_mdp.is_alive, weight=1.0)


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)


@configclass
class SoftDiceTrackingEnvCfg(ManagerBasedRLEnvCfg):
    scene: SoftDiceSceneCfg = SoftDiceSceneCfg(num_envs=1, env_spacing=2.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 1
        self.episode_length_s = 60.0

        # Match the working replay first. We can change control frequency after parity.
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

    def configure_from_motion(
        self,
        motion_file: str,
        *,
        start_frame: int = 0,
        playback_speed: float = 1.0,
        loop: bool = False,
        cube_size: float = DEFAULT_CUBE_SIZE,
        table_length: float = DEFAULT_TABLE_LENGTH,
        table_width: float = DEFAULT_TABLE_WIDTH,
        ground_z: float = DEFAULT_GROUND_Z,
    ) -> None:
        """Configure command timing and initial table/cube layout from a motion file.

        """

        robot_joint_qpos, fps, root_qpos, object_qpos = load_motion_file(motion_file)
        del robot_joint_qpos  # parsed again by MotionCommand when the environment is built

        if root_qpos is None or object_qpos is None:
            raise ValueError(
                "Step-1 soft-dice tracking expects a motion containing root + robot + object qpos (33 columns)."
            )
        if not 0 <= int(start_frame) < root_qpos.shape[0]:
            raise ValueError(f"start_frame={start_frame} outside the motion range.")
        if playback_speed <= 0.0:
            raise ValueError("playback_speed must be > 0.")

        self.commands.motion.motion_file = str(motion_file)
        self.commands.motion.start_frame = int(start_frame)
        self.commands.motion.playback_speed = float(playback_speed)
        self.commands.motion.loop = bool(loop)

        robot_pos = np.asarray(self.scene.robot.init_state.pos, dtype=np.float32)
        robot_quat = np.asarray(self.scene.robot.init_state.rot, dtype=np.float32)

        cube_pos, _, *_ = desired_cube_pose_from_holosoma(
            root_qpos,
            object_qpos,
            int(start_frame),
            robot_pos,
            robot_quat,
            reference_offset=None,
            apply_z_lift=False,
        )

        # Spawn the undeformed asset at the correct center. Orientation is applied by the
        # reset event through the nodal-state API.
        self.scene.cube.init_state.pos = cube_pos.tolist()

        cube_height = float(cube_size) * float(CUSTOM_DICE_SCALE[2])
        cube_bottom_z = float(cube_pos[2] - cube_height / 2.0)
        table_thickness = cube_bottom_z - float(ground_z)
        if table_thickness <= 0.0:
            raise ValueError(f"Computed table thickness is not positive: {table_thickness}")

        table_pos = np.asarray(self.scene.table.init_state.pos, dtype=np.float32)
        table_pos[0] = float(cube_pos[0] - 0.25)
        table_pos[1] = float(cube_pos[1])
        table_pos[2] = float(ground_z + table_thickness / 2.0)

        self.scene.table.init_state.pos = table_pos.tolist()
        self.scene.table.spawn.size = (
            float(table_length),
            float(table_width),
            float(table_thickness),
        )

        # Avoid timing out before the end of a non-looping parity replay.
        if loop:
            self.episode_length_s = 3600.0
        else:
            remaining_frames = max(root_qpos.shape[0] - int(start_frame), 1)
            duration_s = remaining_frames / float(fps) / float(playback_speed)
            self.episode_length_s = duration_s + 1.0
