from launch import LaunchDescription #mange with node, timer, event in launch file
from launch_ros.actions import Node #import action Node for ROS2 to create node from
                                    #package, executable

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agrirobot_core', #package name, contained nodes
            executable='diffdrive_publisher',  # <-- Use executable name from setup.py
            name='motor_publisher', # Node name in ROS2 runtime to mange each node, debug
            output='screen' #show log/text node on terminal
        )
    ])
#source ~/agrirobot_ws/install/setup.bash
#ros2 launch <package_name> <launch_file_name>  
