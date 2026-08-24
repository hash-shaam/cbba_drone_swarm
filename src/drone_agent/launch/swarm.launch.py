from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Shared flock goal -- the "migratory urge". Same for all three drones.
    GOAL_X, GOAL_Y = 12.0, 2.0

    drones = [
        {"id": "drone1", "x": 0.0, "y": 0.0},
        {"id": "drone2", "x": 1.8, "y": 1.2},
        {"id": "drone3", "x": 1.8, "y": -1.2},
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
                    "goal_x": GOAL_X,
                    "goal_y": GOAL_Y,
                }]
            )
        )

    return LaunchDescription(nodes)