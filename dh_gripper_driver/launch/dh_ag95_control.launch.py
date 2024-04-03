from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "com_port",
            default_value="/dev/robot/dh_ag95_gripper",
            description="COM port of the gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mocked command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    com_port = LaunchConfiguration("com_port")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dh_ag95_description"),
                    "urdf",
                    "dh_ag95.urdf.xacro",
                ]
            ),
            " ",
            "com_port:=",
            com_port,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # Load controllers configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dh_gripper_driver"),
            "config",
            "dh_ag95_controllers.yaml",
        ]
    )
    # Load RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dh_gripper_driver"), "rviz", "dh_ag95_control.rviz"]
    )

    # controller manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ('controller_manager/robot_description', 'robot_description'),
        ],
    )
    # robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    # rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # helper function to spawn controllers
    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name] + [a for a in args],
        )

    # Active controllers
    active_list = [
        "joint_state_broadcaster",
        "forward_position_controller",
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]

    # Inactive controllers
    inactive_list = []
    inactive_spawners = [controller_spawner(controller, "--inactive") for controller in inactive_list]

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
    ] + active_spawners + inactive_spawners

    return LaunchDescription(declared_arguments + nodes)