import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    INDEX = int(os.environ.get('PEER_ID')) 

    heartbeat_node = Node(
        package="rov_app",
        namespace=f"drone_{INDEX}",
        executable="heartbeat",
        name='heartbeat',
        parameters=[{
            "peer_index":INDEX
        }]

    )

    movement_node = Node(
        package="rov_app",
        namespace=f"drone_{INDEX}",
        executable="movement",
        name='movement',
        parameters=[{
            "peer_index":INDEX
        }]

    )

    stream_node = Node(
        package="rov_app",
        namespace=f"drone_{INDEX}",
        executable="videostream",
        name='videostream',
        parameters=[{
            "peer_index": INDEX,
            "resolution" : (320,240)
        }]
    )

    # Add the nodes and the process to the LaunchDescription list
    ld = [
        heartbeat_node,
        stream_node,
        movement_node
    ]

    return LaunchDescription(ld)