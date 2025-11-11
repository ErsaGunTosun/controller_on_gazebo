import os 

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    
    launch_path = os.path.join(
            get_package_share_directory("controller_on_gazebo"),
            "launch"
    )
    
    world_path = os.path.join(
            get_package_share_directory("controller_on_gazebo"),
            "worlds",
            "empty_world.world"
    )

    gz_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")
            ),
            launch_arguments ={"gz_args": ["-s -r -v1 ", world_path]}.items()
    )

    gz_client = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")    
            ),
            launch_arguments = {"gz_args": ["-g -v1"]}.items()
    )
    

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_path,"robot_state_publisher.launch.py")
             )
    )

    spawn_entity = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_path,"spawn_entity.launch.py")
            )
    )

    joy = Node(
        package = "joy",
        executable = "joy_node",
        name = "joy_node",  
        parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
        }]
    )



    return LaunchDescription([
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_entity,
        joy
        ])
