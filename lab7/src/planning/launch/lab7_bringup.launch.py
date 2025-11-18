from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.events import Shutdown
from launch.actions import IncludeLaunchDescription  
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # RealSense (include rs_launch.py)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'pointcloud.enable': 'true',
            'rgb_camera.color_profile': '1920x1080x30',
        }.items(),
    )

    # Args for perception node
    
    # A: -0.1383188085182711, B: -0.9869218039475554, C: -0.08278441944586651, D: 0.07161892649501833
    plane_a_launch_arg = DeclareLaunchArgument(
        'plane_a',
        # default_value='0.0'
        # default_value='0.015377132520637958'
        default_value='0.0'
    )
    plane_b_launch_arg = DeclareLaunchArgument(
        'plane_b',
        # default_value='1.0'
        default_value='0.9992937271960843'
    )
    plane_c_launch_arg = DeclareLaunchArgument(
        'plane_c',
        # default_value='0.0'
        # default_value='0.03653863014429065'
        default_value='0.1239079281658973'
    )
    plane_d_launch_arg = DeclareLaunchArgument(
        'plane_d',
        # default_value='-0.075'
        # default_value='-0.07200981472717885'
        default_value='-0.08194036378063227'
    )
    plane_a = LaunchConfiguration('plane_a')
    plane_b = LaunchConfiguration('plane_b')
    plane_c = LaunchConfiguration('plane_c')
    plane_d = LaunchConfiguration('plane_d')


    # Perception node
    perception_node = Node(
        package='perception',
        executable='process_pointcloud',
        name='process_pointcloud',
        output='screen',
        parameters=[{
            'plane.a': plane_a,
            'plane.b': plane_b,
            'plane.c': plane_c,
            'plane.d': plane_d,
        }]
    )
    #transformcube_node
    transform_cube_node= Node(
        package='planning',
        executable='transform_cube_pose',
        name='cube_pose',
        output='screen',
        parameters=[],
    )
    
    ik_node = Node(
        package='planning',
        executable='ik',
        name='ik_node',
        output='screen',
        parameters=[],
    )

    # ArUco recognition
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_aruco'),
                'launch',
                'aruco_recognition.launch.py'
            )
        )
    )

    ar_marker_launch_arg = DeclareLaunchArgument(
        'ar_marker',
        default_value='ar_marker_8'
    )
    ar_marker = LaunchConfiguration('ar_marker')

    # Planning TF node
    planning_tf_node = Node(
        package='planning',
        executable='tf',
        name='tf_node',
        output='screen',
        parameters=[{
            'ar_marker': ar_marker,
        }]
    )

    # Static TF: base_link -> world
    # -------------------------------------------------
    # This TF is static because the "world" frame does not move.
    # It is necessary to define the "world" frame for MoveIt to work properly as this is the defualt planning frame.
    # -------------------------------------------------
    static_base_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_world',
        arguments=['0','0','0','0','0','0','1','base_link','world'],
        output='screen',
    )

    # MoveIt 
    ur_type = LaunchConfiguration("ur_type", default="ur7e")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")

    # Path to the MoveIt launch file
    moveit_launch_file = os.path.join(
                get_package_share_directory("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            )

    # Include the MoveIt launch description
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz
        }.items(),
    )

    # -------------------------
    # Global shutdown on any process exit
    # -------------------------
    shutdown_on_any_exit = RegisterEventHandler(
        OnProcessExit(
            on_exit=[EmitEvent(event=Shutdown(reason='SOMETHING BONKED'))]
        )
    )
    
    return LaunchDescription([
        ar_marker_launch_arg,
        plane_a_launch_arg,
        plane_b_launch_arg,
        plane_c_launch_arg,
        plane_d_launch_arg,
        realsense_launch,
        aruco_launch,
        perception_node,
        planning_tf_node,
        static_base_world,
        moveit_launch,
        shutdown_on_any_exit,
        transform_cube_node,
        ik_node
    ])
