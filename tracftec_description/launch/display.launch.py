from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    trafctec_description = get_package_share_path("trafctec_description")

    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    jsp_gui = LaunchConfiguration("jsp_gui")
    rviz_config = LaunchConfiguration("rviz_config")

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(trafctec_description / "launch" / "robot_state_publisher.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(publish_robot_state),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(jsp_gui),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(jsp_gui),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(trafctec_description / "rviz" / "display.rviz"),
                description="Path to rviz config file",
            ),
            DeclareLaunchArgument(
                "publish_robot_state",
                default_value="true",
                description="Run robot state publisher if true",
            ),
            DeclareLaunchArgument(
                "jsp_gui",
                default_value="true",
                description="Use joint_state_publisher_gui instead of joint_state_publisher if true",
            ),
            robot_state_publisher_launch,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
