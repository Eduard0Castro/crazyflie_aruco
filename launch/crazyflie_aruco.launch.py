from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():

    node_name_param = "crazyflie_aruco_control_node"
    return LaunchDescription([
        LifecycleNode(
            package='crazyflie_aruco',
            executable='crazyflie_aruco_control',
            name=node_name_param,
            namespace="",

        ),
        Node(
            package='crazyflie_aruco',
            executable='control_crazyflie_lifecycle',
            parameters=[{"node_name":node_name_param}]
        ),

    ])