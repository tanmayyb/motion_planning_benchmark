"""
A launch file for running the motion planning python api tutorial
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda", package_name="moveit_resources_panda_moveit_config"
        )
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("benchmark"),
                "config",
                "motion_planning.yaml")
        )
        .to_moveit_configs()
    )

    benchmark_script = DeclareLaunchArgument(
        "benchmark",
        default_value="benchmark.py",
        description="Benchmark plan for the Robot",
    )


    logger_script = DeclareLaunchArgument(
        "logger_script",
        default_value="logger.py",
        description="logging script for logging joint state information",
    )

    logger_node     = Node(
        name="joint_logger_node",
        package="benchmark",
        executable=LaunchConfiguration("logger_script"),
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="benchmark",
        executable=LaunchConfiguration("benchmark"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("benchmark"),
        "config",
        "benchmark.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )

    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]



    return LaunchDescription(
        [
            logger_script,
            logger_node,
            benchmark_script,
            moveit_py_node,
            robot_state_publisher,
            ros2_control_node,
            rviz_node,
            static_tf,
        ]
        + load_controllers
    )