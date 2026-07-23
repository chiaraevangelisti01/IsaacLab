import argparse
import torch

from isaaclab.app import AppLauncher

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=1300)
parser.add_argument("--push_speed", type=float, default=0.03, help="Pusher speed in m/s (world frame).")
parser.add_argument("--push_duration", type=float, default=240.00, help="How long to push (seconds).")
parser.add_argument("--push_y_offset", type=float, default=0.30, help="Initial y offset from cube center (m).")
parser.add_argument("--x_offcenter", type=float, default=0.10, help="Off-center x offset to induce yaw (m).")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# -----------------------------------------------------------------------------
# Isaac Lab imports
# -----------------------------------------------------------------------------
import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.assets import AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.sim.simulation_cfg import PhysxCfg



# -----------------------------------------------------------------------------
# Scene Config
# -----------------------------------------------------------------------------
@configclass
class SpawningSceneCfg(InteractiveSceneCfg):
    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0.0]),
        spawn=GroundPlaneCfg(),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # Table (USD) -- same as yours
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.2, 0, 1.0], rot=[0.707, 0, 0, -0.707]),
        spawn=UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"
        ),
    )

    # Deformable Cube -- same as yours
    cube = DeformableObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.MeshCuboidCfg(
            size=(0.30, 0.30, 0.30),
            deformable_props = sim_utils.DeformableBodyPropertiesCfg(
                deformable_enabled=True,
                kinematic_enabled=False,
                self_collision=True,
                simulation_hexahedral_resolution=4,  # default 10
                collision_simplification=True,
                collision_simplification_remeshing=True,
                collision_simplification_remeshing_resolution=30,
                collision_simplification_target_triangle_count=0,
                collision_simplification_force_conforming=True,
                contact_offset=0.008,
                rest_offset=0.004,
                solver_position_iteration_count=128,
                vertex_velocity_damping= 5.0,
                settling_threshold = 0.05,
                sleep_threshold = 0.02,
                sleep_damping = 10.0,

              
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.1, 0.0)),
            physics_material=sim_utils.DeformableBodyMaterialCfg(
                poissons_ratio=0.2,
                youngs_modulus=9.5e3,
                density = 20.5,
                dynamic_friction=0.75,
                elasticity_damping = 0.05,   # default is VERY low
                damping_scale = 1.0         # keep or slightly increase

            ),
        ),
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.15)),
        debug_vis=True,
    )

    # KINEMATIC PUSHER (rigid cuboid)
    # - Kinematic means we "teleport" it by setting its pose each step.
    # - disable_gravity True so it does not fall.
    # - collision_props enabled so it collides with the cube.

    pusher = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Pusher",
        spawn=sim_utils.CapsuleCfg(
            radius=0.035,          # 7 cm diameter-ish (tune)
            height=0.18,           # capsule cylinder part length (tune)
            axis="X",              # capsule's long axis ("X","Y","Z")
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=True,   # position-controlled rigid body [3](https://isaac-sim.github.io/IsaacLab/main/source/how-to/make_fixed_prim.html)
                disable_gravity=True,
                max_linear_velocity=2.0,
                max_angular_velocity=10.0,
                max_depenetration_velocity=2.0,
                solver_position_iteration_count=64,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=3.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0,
                dynamic_friction=1.0,
                restitution=0.0,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.4, 0.9), metallic=0.2),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.4, 1.15),
            rot=(1.0, 0.0, 0.0, 0.0)  # (w,x,y,z)
        ),
    )

physx = PhysxCfg(
    bounce_threshold_velocity=0.2,
    gpu_max_rigid_contact_count=2**20,      # default 2**23
    gpu_max_rigid_patch_count=2**18,        # default 5 * 2**15
    gpu_temp_buffer_capacity=2**20,         # default 2**20
    gpu_max_soft_body_contacts=2**24,       # default 2**20
    gpu_collision_stack_size=2**30,         # default 2**26
    min_position_iteration_count=64,
)




# -----------------------------------------------------------------------------
# Simulation logic
# -----------------------------------------------------------------------------
def run(sim: SimulationContext, scene: InteractiveScene):
    cube = scene["cube"]
    pusher = scene["pusher"]

    #sim_dt = sim.get_physics_dt()
    sim_dt = 1/90.0  
    steps = int(args_cli.steps)

    # Identity quaternion (w,x,y,z)
    quat_w = torch.tensor([1.0, 0.0, 0.0, 0.0], device=sim.device).repeat(scene.num_envs, 1)

    # Pre-step update so buffers are valid
    scene.update(sim_dt)

    # Read cube "root" position (Isaac Lab exposes this buffer in DeformableObjectData).
    cube_pos_w = cube.data.root_pos_w.clone()  # (N,3)
    # Place pusher relative to cube:
    # - start at +Y side
    # - slightly off-center in +X to induce yaw
    start_pos = cube_pos_w.clone()
    start_pos[:, 0] += float(args_cli.x_offcenter)
    start_pos[:, 1] += float(args_cli.push_y_offset)
    start_pos[:, 2] = cube_pos_w[:, 2]  # push around cube center height

    # Write initial pusher pose to sim (real API method)
    # Note: write_root_pose_to_sim expects (N,7): [x,y,z,qw,qx,qy,qz]. [1](https://isaac-sim.github.io/IsaacLab/v2.1.1/_modules/isaaclab/assets/rigid_object/rigid_object.html)
    pusher_pose = torch.cat([start_pos, quat_w], dim=1)
    pusher.write_root_pose_to_sim(pusher_pose)  # [1](https://isaac-sim.github.io/IsaacLab/v2.1.1/_modules/isaaclab/assets/rigid_object/rigid_object.html)

    # Push trajectory: move along -Y for push_duration seconds, then hold.
    push_steps = max(1, int(args_cli.push_duration / sim_dt))
    push_steps = min(push_steps, steps)
    v = float(args_cli.push_speed)  # m/s
    dy_per_step = v * sim_dt

    # Simple diagnostic threshold: if pusher center goes beyond cube center by more than half cube size,
    # we consider it "deep penetration". This is a coarse heuristic (not a true signed distance).
    cube_half = 0.30 * 0.5

    count = 0
    while simulation_app.is_running() and count < steps:
        # Update cube position each step (cube might move)
        cube_pos_w = cube.data.root_pos_w.clone()

        # Compute desired pusher position
        p_pos = start_pos.clone()
        if count < push_steps:
            p_pos[:, 1] = start_pos[:, 1] - dy_per_step * count
        else:
            p_pos[:, 1] = start_pos[:, 1] - dy_per_step * push_steps

        # Keep same z (you can later add a slight z offset if you want to push higher/lower)
        p_pos[:, 2] = cube_pos_w[:, 2]
        p_pose = torch.cat([p_pos, quat_w], dim=1)

        # Apply pusher pose into sim
        pusher.write_root_pose_to_sim(p_pose)  # [1](https://isaac-sim.github.io/IsaacLab/v2.1.1/_modules/isaaclab/assets/rigid_object/rigid_object.html)

        # Step sim
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        # Very lightweight penetration heuristic print every 60 frames
        if count % 60 == 0:
            # If pusher center is within cube "core" along Y by more than half-size, it's suspicious.
            # (Again: heuristic only.)
            delta_y = (p_pos[:, 1] - cube_pos_w[:, 1]).abs()
            suspicious = (delta_y < (cube_half * 0.2))  # pusher center near cube center
            print(
                f"[step {count}] cube_y={cube_pos_w[0,1].item():+.3f} "
                f"pusher_y={p_pos[0,1].item():+.3f} "
                f"|dy|={delta_y[0].item():.3f} "
                f"suspicious={bool(suspicious[0].item())}"
            )

        count += 1



def main():
    # IMPORTANT: deformables require GPU simulation in Isaac Lab tutorials;
    # you already run on GPU, so we keep the SimulationCfg as you did.
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, physx=physx)
    sim = SimulationContext(sim_cfg)

    sim.set_camera_view([1.0, 0.0, 2.5], [0.0, 0.0, 1.0])

    scene_cfg = SpawningSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.5)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO]: Scene ready! Running pusher test...")

    run(sim, scene)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()