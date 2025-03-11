"""Configuration for frog model v0 """


import isaaclab.sim as sim_utils
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.actuators import DCMotorCfg, IdealPDActuatorCfg

##
# Configuration
##
FROG_V0_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/data/cevangel/IsaacLab/source/isaaclab_assets/data/robots/frog/frogv0.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0,
            angular_damping=0, 
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.4),
        joint_pos={".*": 0.0},
        # #     "(L|R)_elbow": 0,
        #     #   ".*_hip_.*": 0.5,
        # },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.95, #to check, probably not needed since there is no hardware
    actuators={
        "base_legs": DCMotorCfg(
            joint_names_expr=[".*hip.*", ".*knee1.*", ".*knee2.*"],
            effort_limit=0.01,
            saturation_effort=0.01,
            velocity_limit=10,
            stiffness=1e-3,
            damping=1e-4,
            friction=0.0,
        ),
        "base_arms": DCMotorCfg( # I would define them with lower effort
            joint_names_expr=[".*shoulder.*",".*elbow.*"],
            effort_limit=0.01,
            saturation_effort=0.01,
            velocity_limit=10,
            stiffness=1e-3,
            damping=1e-4,
            friction=0.0,
        ),
    },
)