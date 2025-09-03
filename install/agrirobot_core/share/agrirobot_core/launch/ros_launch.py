from launch import LaunchDescription #mange with node, timer, event in launch file
from launch_ros.actions import Node #import action Node for ROS2 to create node from
                                    #package, executable

Left_node =Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            output="screen",
            arguments=["serial", "-D", "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0", "-b", "115200", "-v6"],
            emulate_tty=True,
        )
Right_node = Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            output="screen",
            arguments=["serial", "-D", "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0", "-b", "115200", "-v6"],
            emulate_tty=True,
        )

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='agrirobot_core', #package name, contained nodes
            executable='diffdrive_publisher',  # <-- Use executable name from setup.py
            name='motor_publisher', # Node name in ROS2 runtime to mange each node, debug
            output='screen' #show log/text node on terminal
        ),
        #- Two micro-ROS Agents (one per serial port)
        Left_node,
        #Right_node
    ])

#ros2 run micro_ros_agent micro_ros_agent serial   -D /dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0   -b 115200   -v6
#ros2 run micro_ros_agent micro_ros_agent serial   -D /dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0   -b 115200   -v6
#source ~/agrirobot_ws/install/setup.bash
#ros2 launch <package_name> <launch_file_name>  
