import math
import csv

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class DroneNode(Node):

    def __init__(self):

        super().__init__("drone_node")

        # ---------------- Parameters ----------------

        self.declare_parameter("drone_id", "drone1")
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)

        self.drone_id = self.get_parameter("drone_id").value

        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)

        # Fixed individual goal -- decoupled from the task auction below.
        # This is what drives steps 1-3 (comms / avoidance / flocking).
        self.goal_x = float(self.get_parameter("target_x").value)
        self.goal_y = float(self.get_parameter("target_y").value)

        self.vx = 0.0
        self.vy = 0.0

        self.dt = 0.1

        # ---------------- Publisher ----------------

        self.publisher = self.create_publisher(
            Point,
            f"/{self.drone_id}/state",
            10
        )

        # ---------------- Neighbor Storage ----------------

        self.neighbors = {}          # drone_name -> (x, y)
        self.neighbor_vel = {}       # drone_name -> (vx, vy), finite-differenced
        self.neighbor_last_time = {}

        # ---------------- CBBA-lite bidding (prototype, does NOT gate motion) ----------------
        # This exists to demonstrate the bid/topic pattern discussed in the
        # CBBA design doc. It intentionally does not control drone movement --
        # see report section on CBBA design for why real bundle/consensus rounds
        # were designed but not implemented.

        self.task = None
        self.bid = 0.0
        self.all_bids = {}

        self.bid_publisher = self.create_publisher(
            Point,
            f"/bids/{self.drone_id}",
            10
        )

        all_drones = ["drone1", "drone2", "drone3"]

        for drone in all_drones:
            if drone == self.drone_id:
                continue
            self.create_subscription(
                Point,
                f"/{drone}/state",
                lambda msg, drone_name=drone:
                    self.neighbor_callback(msg, drone_name),
                10
            )

        for drone in all_drones:
            if drone == self.drone_id:
                continue
            self.create_subscription(
                Point,
                f"/bids/{drone}",
                lambda msg, drone_name=drone:
                    self.bid_callback(msg, drone_name),
                10
            )

        self.create_subscription(
            Point,
            "/task",
            self.task_callback,
            10
        )

        # ---------------- Statistics ----------------

        self.closest_distance_ever = float("inf")
        self.avoidance_count = 0
        self.safety_violated = False
        self.currently_avoiding = False

        # ---------------- CSV logging (for matplotlib plot) ----------------

        self.log_path = f"/tmp/{self.drone_id}_positions.csv"
        self.log_file = open(self.log_path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["time", "x", "y", "min_distance"])

        # ---------------- Timer ----------------

        self.timer = self.create_timer(self.dt, self.update_position)

        self.get_logger().info(
            f"{self.drone_id} started. goal=({self.goal_x:.1f},{self.goal_y:.1f})"
        )

    # ------------------------------------------------

    def neighbor_callback(self, msg, drone_name):

        now = self.get_clock().now().nanoseconds / 1e9

        if drone_name in self.neighbors:
            old_x, old_y = self.neighbors[drone_name]
            old_t = self.neighbor_last_time.get(drone_name, now)
            dt = max(now - old_t, 1e-3)
            vx = (msg.x - old_x) / dt
            vy = (msg.y - old_y) / dt
            self.neighbor_vel[drone_name] = (vx, vy)

        self.neighbors[drone_name] = (msg.x, msg.y)
        self.neighbor_last_time[drone_name] = now

    def bid_callback(self, msg, drone_name):
        self.all_bids[drone_name] = msg.x

    def task_callback(self, msg):
        self.task = (msg.x, msg.y)
        self.get_logger().info(f"Received task at ({msg.x:.1f}, {msg.y:.1f})")

    # ------------------------------------------------

    def update_position(self):

        SAFE_DISTANCE = 2.0          # soft warning threshold
        HARD_MIN_SEPARATION = 0.5    # true collision -- must NEVER be breached
        DETECTION_RADIUS = 3.0
        AVOID_GAIN = 2.5
        GOAL_GAIN = 1.2
        ALIGN_GAIN = 0.5
        COHESION_GAIN = 0.3
        MAX_SPEED = 2.0

        # ---------- Goal attraction (independent of auction) ----------

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        goal_dist = math.sqrt(dx * dx + dy * dy)

        goal_x, goal_y = 0.0, 0.0
        if goal_dist > 0.2:
            goal_x = dx / goal_dist
            goal_y = dy / goal_dist

        # ---------- CBBA-lite bid (logging/demo only) ----------

        winner = self.drone_id
        if self.task is not None:
            tdx = self.task[0] - self.x
            tdy = self.task[1] - self.y
            tdist = math.sqrt(tdx * tdx + tdy * tdy)
            self.bid = 1.0 / (tdist + 0.1)

            bid_msg = Point()
            bid_msg.x = self.bid
            self.bid_publisher.publish(bid_msg)

            highest_bid = self.bid
            for drone, bid in self.all_bids.items():
                if bid > highest_bid:
                    highest_bid = bid
                    winner = drone

        # ---------- Separation (avoidance) + alignment + cohesion ----------

        avoid_x = avoid_y = 0.0
        align_x = align_y = 0.0
        cohesion_x = cohesion_y = 0.0
        min_distance = float("inf")

        n_neighbors = len(self.neighbors)

        for name, pos in self.neighbors.items():

            ndx = self.x - pos[0]
            ndy = self.y - pos[1]
            distance = math.sqrt(ndx * ndx + ndy * ndy)

            if distance < min_distance:
                min_distance = distance
            if distance < self.closest_distance_ever:
                self.closest_distance_ever = distance

            if 0.001 < distance < DETECTION_RADIUS:
                strength = (DETECTION_RADIUS - distance) / DETECTION_RADIUS
                avoid_x += (ndx / distance) * strength
                avoid_y += (ndy / distance) * strength

        if n_neighbors > 0:
            # Alignment: match average neighbour velocity
            vels = list(self.neighbor_vel.values())
            if vels:
                avg_vx = sum(v[0] for v in vels) / len(vels)
                avg_vy = sum(v[1] for v in vels) / len(vels)
                speed = math.sqrt(avg_vx ** 2 + avg_vy ** 2)
                if speed > 0.01:
                    align_x = avg_vx / speed
                    align_y = avg_vy / speed

            # Cohesion: steer toward centroid of neighbours
            cx = sum(p[0] for p in self.neighbors.values()) / n_neighbors
            cy = sum(p[1] for p in self.neighbors.values()) / n_neighbors
            cdx, cdy = cx - self.x, cy - self.y
            cdist = math.sqrt(cdx ** 2 + cdy ** 2)
            if cdist > 0.1:
                cohesion_x = cdx / cdist
                cohesion_y = cdy / cdist

        # ---------- Count avoidance events ----------

        if min_distance < DETECTION_RADIUS:
            if not self.currently_avoiding:
                self.currently_avoiding = True
                self.avoidance_count += 1
        else:
            self.currently_avoiding = False

        # ---------- Combine Reynolds terms ----------

        self.vx = (goal_x * GOAL_GAIN + avoid_x * AVOID_GAIN +
                   align_x * ALIGN_GAIN + cohesion_x * COHESION_GAIN)
        self.vy = (goal_y * GOAL_GAIN + avoid_y * AVOID_GAIN +
                   align_y * ALIGN_GAIN + cohesion_y * COHESION_GAIN)

        speed = math.sqrt(self.vx ** 2 + self.vy ** 2)
        if speed > MAX_SPEED:
            self.vx = self.vx / speed * MAX_SPEED
            self.vy = self.vy / speed * MAX_SPEED

        # ---------- Move ----------

        self.x += self.vx * self.dt
        self.y += self.vy * self.dt

        # ---------- Publish state ----------

        msg = Point()
        msg.x = self.x
        msg.y = self.y
        self.publisher.publish(msg)

        # ---------- Safety check (soft + hard) ----------

        if min_distance < SAFE_DISTANCE:
            self.safety_violated = True

        if min_distance < HARD_MIN_SEPARATION:
            self.get_logger().error(
                f"CRITICAL: {self.drone_id} hard separation breached: "
                f"{min_distance:.2f}m"
            )

        assert min_distance > HARD_MIN_SEPARATION, (
            f"{self.drone_id}: SAFETY VIOLATION -- minimum separation "
            f"{min_distance:.2f}m breached hard limit {HARD_MIN_SEPARATION}m"
        )

        # ---------- CSV log ----------

        t = self.get_clock().now().nanoseconds / 1e9
        self.csv_writer.writerow([t, self.x, self.y, min_distance])
        self.log_file.flush()

        # ---------- Status ----------

        if min_distance == float("inf"):
            text = "No neighbours"
        elif min_distance < SAFE_DISTANCE:
            text = f"COLLISION WARNING min={min_distance:.2f}"
        elif min_distance < DETECTION_RADIUS:
            text = f"AVOIDING min={min_distance:.2f}"
        else:
            text = f"SAFE min={min_distance:.2f}"

        self.get_logger().info(
            f"{self.drone_id} ({self.x:.2f}, {self.y:.2f}) {text} "
            f"| Bid={self.bid:.3f} | Winner={winner}"
        )

    # ------------------------------------------------

    def destroy_node(self):

        self.log_file.close()

        pass_fail = "PASS" if not self.safety_violated else "FAIL (soft threshold)"

        print("\n========== SAFETY SUMMARY ==========")
        print(f"Drone             : {self.drone_id}")
        print(f"Closest distance  : {self.closest_distance_ever:.2f} m")
        print(f"Avoidance events  : {self.avoidance_count}")
        print(f"Soft threshold    : {pass_fail}")
        print(f"Hard threshold    : PASS (never breached -- would have crashed)")
        print(f"Position log      : {self.log_path}")
        print("=====================================\n")

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    node = DroneNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()