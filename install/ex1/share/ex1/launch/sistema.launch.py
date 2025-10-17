from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodos publicadores
        Node(
            package='ex1',
            executable='node1',
            name='node_1'
        ),
        Node(
            package='ex1',
            executable='node2',
            name='node_2'
        ),
        Node(
            package='ex1',
            executable='node3',
            name='node_3'
        ),

        # Nodo que calcula promedio
        Node(
            package='ex1',
            executable='node4',
            name='node_4'
        ),

        # Nodo que muestra el promedio
        Node(
            package='ex1',
            executable='node5',
            name='node_5'
        ),
    ])
