from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    drones = [
        # drone1 and drone2 are head-on -- guaranteed to force avoidance
        {"id": "drone1", "x": 0.0, "y": 0.0,  "tx": 10.0, "ty": 0.0},
        {"id": "drone2", "x": 10.0, "y": 0.0, "tx": 0.0,  "ty": 0.0},
        # drone3 crosses perpendicular through the middle
        {"id": "drone3", "x": 5.0, "y": 6.0,  "tx": 5.0,  "ty": -6.0},
    ]

    nodes = []
    for d in drones:
        nodes.append(
            Node(
                package="drone_agent",
                executable="drone_node",
                name=d["id"],
                parameters=[{
                    "drone_id": d["id"],
                    "start_x": d["x"],
                    "start_y": d["y"],
                    "target_x": d["tx"],
                    "target_y": d["ty"],
                }]
            )
        )

    return LaunchDescription(nodes)