from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Declare a variable Node for each node
    INDEX = 0
    VERBOSE = False

    heartbeat_node = Node(
        package="rov_app",
        namespace=f"drone_{INDEX}",
        executable="heartbeat",
        name='heartbeat',
        parameters=[{
            "verbose" : VERBOSE,
            "peer_index":INDEX
        }]


    )


    movement_node = Node(
        package="rov_app",
        namespace=f"drone_{INDEX}",
        executable="movement",
        name='movement',
        parameters=[{
            "verbose" : VERBOSE,
            "peer_index":INDEX
        }]

    )

    stream_node = Node(
        package="rov_app",
        namespace=f"drone_{INDEX}",
        executable="videostream",
        name='videostream',
        parameters=[{
            "verbose" : VERBOSE,
            "peer_index": INDEX,
            "framerate" : 30,
            "resolution" : (640,480),
            "opencv_render" : True

        }]
    )

    # Add the nodes and the process to the LaunchDescription list
    ld = [
        heartbeat_node,
        stream_node,
        movement_node,
    ]

    return LaunchDescription(ld)