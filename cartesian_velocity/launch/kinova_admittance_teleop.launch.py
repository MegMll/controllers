from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_simulation = LaunchConfiguration("use_simulation")
    use_fake_hardware = PythonExpression([
            "'true' if '", use_simulation, "' == 'true' else 'false'"
        ])
    fake_sensor_commands = PythonExpression([
            "'true' if '", use_simulation, "' == 'true' else 'false'"
        ])
    
    robot_ip = LaunchConfiguration("robot_ip")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_simulation", 
            default_value="false", 
            description="Start robot in simulation."
        ),
        DeclareLaunchArgument(
            "robot_ip", 
            default_value="dont-care",
            description="IP address by which the robot can be reached."
        ),
    ]

    urdf_cmd = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kortex_description"), "robots", "gen3.xacro"]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "name:=",
            "arm",
            " ",
            "arm:=",
            "gen3",
            " ",
            "dof:=7",
            " ",
            "prefix:=",
            "",
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "gripper:=",
            "robotiq_2f_85",
            " ",
            "use_internal_bus_gripper_comm:=",
            "true",
            " ",
            "gripper_max_velocity:=",
            "100.0",
            " ",
            "gripper_max_force:=",
            "100.0",
            " ",
            "gripper_joint_name:=",
            "robotiq_85_left_knuckle_joint",
            " ",
            "use_external_cable:=",
            "true"
        ]
    )
    robot_description = {"robot_description": urdf_cmd}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    velocity_config = PathJoinSubstitution([
        FindPackageShare("cartesian_velocity"), "config", "kinova_params_with_admittance.yaml"
    ])

    # --------------------------------------------------------------------------
    # Controllers spawner
    # --------------------------------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, velocity_config],
        output="both",
    )

    spawner_qontrol = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=["qontrol_explorer", "--controller-manager", "/controller_manager"],
    )

    spawner_observer_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["generalized_momentum_observer", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_teleop_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_velocity_teleop_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--activate",
        ],
        output="screen",
    )

    # Spawner for robotiq_gripper_controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # --------------------------------------------------------------------------
    # Other Nodes
    # --------------------------------------------------------------------------
    # --- Teleoperation Node ---
    teleop_config_file = PathJoinSubstitution(
        [
            FindPackageShare("joystick_interface"),
            "config",
            "kinova_joystick_parameters.yaml",
        ]
    )

    teleop_node = Node(
        package="joystick_interface",
        executable="joystick_input_node",
        name="joystick_input_node",
        output="screen",
        parameters=[teleop_config_file]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node"
    )
    # --------------------------------------------------------------------------
    # Event Handlers
    # --------------------------------------------------------------------------
    delayed_spawner_qontrol = TimerAction(
        period=2.0,
        actions=[spawner_qontrol]
    )
    start_observer_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_qontrol,
            on_exit=[spawner_observer_controller, gripper_controller_spawner] #, fault_controller_spawner]
        )
    )

    start_teleop_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_observer_controller,
            on_exit=[spawner_teleop_controller] #, fault_controller_spawner]
        )
    )

    # --------------------------------------------------------------------------
    # Launch Description
    # --------------------------------------------------------------------------
    nodes_to_start = [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delayed_spawner_qontrol,
        start_observer_event,
        start_teleop_event,
        teleop_node,
        joy_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)