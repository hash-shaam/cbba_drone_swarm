import math
import csv

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class DroneNode(Node):

    def __init__(self):

        super().__init__("drone_node")

        self.declare_parameter("drone_id", "drone1")
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)

        self.drone_id = self.get_parameter("drone_id").value

        self.x = float(self.get_parameter("start_x").value)
        self.y = float(self.get_parameter("start_y").value)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)

        self.vx = 0.0
        self.vy = 0.0
        self.dt = 0.1

        # 0 = default launch goal, not a numbered task. Set to 1/2/3 on task receipt.
        self.current_task_id = 0
        self.holding = False

        # in __init__, add a new state flag:
        self.hold_event_written = False

        # ---------------- Publishers ----------------

        self.publisher = self.create_publisher(Point, f"/{self.drone_id}/state", 10)
        self.vel_publisher = self.create_publisher(Point, f"/{self.drone_id}/velocity", 10)

        # ---------------- Neighbor Storage ----------------

        self.neighbors = {}
        self.neighbor_vel = {}

        # ---------------- CBBA-lite bidding (demo only, NOT implemented) ----------------

        self.bid = 0.0
        self.all_bids = {}
        self.bid_publisher = self.create_publisher(Point, f"/bids/{self.drone_id}", 10)

        all_drones = ["drone1", "drone2", "drone3"]

        for drone in all_drones:
            if drone == self.drone_id:
                continue
            self.create_subscription(
                Point, f"/{drone}/state",
                lambda msg, drone_name=drone: self.neighbor_callback(msg, drone_name),
                10
            )
        for drone in all_drones:
            if drone == self.drone_id:
                continue
            self.create_subscription(
                Point, f"/{drone}/velocity",
                lambda msg, drone_name=drone: self.neighbor_vel_callback(msg, drone_name),
                10
            )
        for drone in all_drones:
            if drone == self.drone_id:
                continue
            self.create_subscription(
                Point, f"/bids/{drone}",
                lambda msg, drone_name=drone: self.bid_callback(msg, drone_name),
                10
            )

        self.create_subscription(Point, "/task", self.task_callback, 10)

        # ---------------- Statistics ----------------

        self.closest_distance_ever = float("inf")
        self.avoidance_count = 0
        self.safety_violated = False
        self.currently_avoiding = False

        # ---------------- CSV logging ----------------

        self.log_path = f"/tmp/{self.drone_id}_positions.csv"
        self.log_file = open(self.log_path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(["time", "x", "y", "min_distance", "alignment_error_deg", "cohesion_error_m"])

        # Arrival events -- one row per task actually reached and held.
        self.events_path = f"/tmp/{self.drone_id}_events.csv"
        self.events_file = open(self.events_path, "w", newline="")
        self.events_writer = csv.writer(self.events_file)
        self.events_writer.writerow(["task_id", "time", "x", "y"])

        # ---------------- Timer ----------------

        self.timer = self.create_timer(self.dt, self.update_position)

        self.get_logger().info(
            f"{self.drone_id} started as part of flock. goal=({self.goal_x:.1f},{self.goal_y:.1f})"
        )

    # ------------------------------------------------

    def neighbor_callback(self, msg, drone_name):
        self.neighbors[drone_name] = (msg.x, msg.y)

    def neighbor_vel_callback(self, msg, drone_name):
        self.neighbor_vel[drone_name] = (msg.x, msg.y)

    def bid_callback(self, msg, drone_name):
        self.all_bids[drone_name] = msg.x

    # in task_callback, reset it whenever a new task arrives:
    def task_callback(self, msg):
        if (self.goal_x, self.goal_y) != (msg.x, msg.y):
            task_id = int(msg.z)
            self.get_logger().info(f"{self.drone_id}: flock redirecting to task {task_id} ({msg.x:.1f}, {msg.y:.1f})")
            self.goal_x = msg.x
            self.goal_y = msg.y
            self.current_task_id = task_id
            self.holding = False
            self.hold_event_written = False   # <-- new task, allow a fresh event log

    # ------------------------------------------------

    def update_position(self):

        SAFE_DISTANCE = 2.0
        HARD_MIN_SEPARATION = 0.5
        DETECTION_RADIUS = 3.0

        SEPARATION_GAIN = 3.0
        ALIGNMENT_GAIN = 1.8
        COHESION_GAIN = 0.7
        GOAL_GAIN = 0.8
        MAX_ACCEL = 3.0
        MAX_SPEED = 2.0

        CAPTURE_RADIUS = 2.0    # "reached" the task/goal
        RELEASE_RADIUS = 4.0    # hysteresis -- only re-engage travel if pushed out this far
        DRAG_COEFF = 0.45       # ambient damping, always active
        HOLD_DAMPING = 3.5      # heavy damping once holding -- overdamped, no overshoot
        HOLD_SPRING_GAIN = 1.0  # distance-proportional pull back to exact goal while holding

        # ---------- Distance to current goal ----------

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        goal_dist = math.sqrt(dx * dx + dy * dy)

        goal_x, goal_y = 0.0, 0.0
        if goal_dist > 0.2:
            goal_x, goal_y = dx / goal_dist, dy / goal_dist

        # ---------- Bid (demo only, does not touch goal) ----------

        tdx = self.goal_x - self.x
        tdy = self.goal_y - self.y
        self.bid = 1.0 / (math.sqrt(tdx * tdx + tdy * tdy) + 0.1)
        bid_msg = Point()
        bid_msg.x = self.bid
        self.bid_publisher.publish(bid_msg)
        winner = self.drone_id
        highest = self.bid
        for d, b in self.all_bids.items():
            if b > highest:
                highest, winner = b, d

        # ---------- Separation, alignment, cohesion ----------

        sep_x = sep_y = 0.0
        align_x = align_y = 0.0
        cohesion_x = cohesion_y = 0.0
        min_distance = float("inf")
        n = len(self.neighbors)

        avg_vx = avg_vy = 0.0
        cx = self.x
        cy = self.y
        has_vel_data = False

        for name, pos in self.neighbors.items():
            ndx, ndy = self.x - pos[0], self.y - pos[1]
            distance = math.sqrt(ndx * ndx + ndy * ndy)
            if distance < min_distance:
                min_distance = distance
            if distance < self.closest_distance_ever:
                self.closest_distance_ever = distance
            if 0.001 < distance < DETECTION_RADIUS:
                strength = (DETECTION_RADIUS - distance) / DETECTION_RADIUS
                sep_x += (ndx / distance) * strength
                sep_y += (ndy / distance) * strength

        if n > 0:
            vels = list(self.neighbor_vel.values())
            if vels:
                avg_vx = sum(v[0] for v in vels) / len(vels)
                avg_vy = sum(v[1] for v in vels) / len(vels)
                speed_v = math.sqrt(avg_vx ** 2 + avg_vy ** 2)
                if speed_v > 0.01:
                    align_x, align_y = avg_vx / speed_v, avg_vy / speed_v
                    has_vel_data = True

            cx = sum(p[0] for p in self.neighbors.values()) / n
            cy = sum(p[1] for p in self.neighbors.values()) / n
            cdx, cdy = cx - self.x, cy - self.y
            cdist = math.sqrt(cdx ** 2 + cdy ** 2)
            if cdist > 0.1:
                cohesion_x, cohesion_y = cdx / cdist, cdy / cdist

        if min_distance < DETECTION_RADIUS:
            if not self.currently_avoiding:
                self.currently_avoiding = True
                self.avoidance_count += 1
        else:
            self.currently_avoiding = False

    # ---------- Arrival state machine ----------

        if not self.holding and goal_dist < CAPTURE_RADIUS:
            self.holding = True
            if self.current_task_id >= 1:
                self.get_logger().info(f"{self.drone_id}: holding at task {self.current_task_id}")
        elif self.holding and goal_dist > RELEASE_RADIUS:
            self.holding = False
            self.hold_event_written = False

        # Log the event once the drone has actually settled near the exact goal
        # point (not just entered the capture radius) — this is what makes the
        # diamond marker land on the real holding position instead of wherever
        # the drone happened to be when it first crossed into range.
        SETTLE_DISTANCE = 0.3   # how close to the exact goal counts as "settled"
        SETTLE_SPEED = 0.2      # and how slow, so it's not just passing through

        if (self.holding and self.current_task_id >= 1 and not self.hold_event_written
                and goal_dist < SETTLE_DISTANCE
                and math.sqrt(self.vx ** 2 + self.vy ** 2) < SETTLE_SPEED):
            t_arrive = self.get_clock().now().nanoseconds / 1e9
            self.events_writer.writerow([self.current_task_id, t_arrive, self.x, self.y])
            self.events_file.flush()
            self.hold_event_written = True

        # ---------- Combine forces (branch on state) ----------

        if self.holding:
            # Spring pull back to the exact goal point (proportional, not unit vector) +
            # separation only (safety). No alignment/cohesion -- don't get dragged by
            # neighbours who haven't settled yet.
            accel_x = dx * HOLD_SPRING_GAIN + sep_x * SEPARATION_GAIN
            accel_y = dy * HOLD_SPRING_GAIN + sep_y * SEPARATION_GAIN
            damping = HOLD_DAMPING
        else:
            accel_x = (goal_x * GOAL_GAIN + sep_x * SEPARATION_GAIN +
                       align_x * ALIGNMENT_GAIN + cohesion_x * COHESION_GAIN)
            accel_y = (goal_y * GOAL_GAIN + sep_y * SEPARATION_GAIN +
                       align_y * ALIGNMENT_GAIN + cohesion_y * COHESION_GAIN)
            damping = DRAG_COEFF

        # Damping opposes current velocity -- this is what lets the system lose energy
        # and actually come to rest, instead of orbiting forever.
        accel_x -= damping * self.vx
        accel_y -= damping * self.vy

        accel_mag = math.sqrt(accel_x ** 2 + accel_y ** 2)
        if accel_mag > MAX_ACCEL:
            accel_x = accel_x / accel_mag * MAX_ACCEL
            accel_y = accel_y / accel_mag * MAX_ACCEL

        self.vx += accel_x * self.dt
        self.vy += accel_y * self.dt

        speed = math.sqrt(self.vx ** 2 + self.vy ** 2)
        if speed > MAX_SPEED:
            self.vx = self.vx / speed * MAX_SPEED
            self.vy = self.vy / speed * MAX_SPEED

        self.x += self.vx * self.dt
        self.y += self.vy * self.dt

        # ---------- Alignment / cohesion error (evidence metrics) ----------

        alignment_error_deg = 0.0
        MIN_SPEED_FOR_HEADING = 0.15   # below this, "heading" is meaningless noise
        self_speed = math.sqrt(self.vx ** 2 + self.vy ** 2)
        if n > 0 and has_vel_data and self_speed > MIN_SPEED_FOR_HEADING:
            my_heading = math.atan2(self.vy, self.vx)
            flock_heading = math.atan2(avg_vy, avg_vx)
            diff = abs(my_heading - flock_heading)
            if diff > math.pi:
                diff = 2 * math.pi - diff
            alignment_error_deg = math.degrees(diff)

        cohesion_error = 0.0
        if n > 0:
            cohesion_error = math.sqrt((self.x - cx) ** 2 + (self.y - cy) ** 2)

        # ---------- Publish state + velocity ----------

        msg = Point()
        msg.x, msg.y = self.x, self.y
        self.publisher.publish(msg)

        vel_msg = Point()
        vel_msg.x, vel_msg.y = self.vx, self.vy
        self.vel_publisher.publish(vel_msg)

        if min_distance < SAFE_DISTANCE:
            self.safety_violated = True
        if min_distance < HARD_MIN_SEPARATION:
            self.get_logger().error(f"CRITICAL: {self.drone_id} hard separation breached: {min_distance:.2f}m")
        assert min_distance > HARD_MIN_SEPARATION, (
            f"{self.drone_id}: SAFETY VIOLATION -- min separation {min_distance:.2f}m breached {HARD_MIN_SEPARATION}m"
        )

        t = self.get_clock().now().nanoseconds / 1e9
        self.csv_writer.writerow([t, self.x, self.y, min_distance, alignment_error_deg, cohesion_error])
        self.log_file.flush()

        if self.holding:
            text = f"HOLDING task{self.current_task_id} min={min_distance:.2f}"
        elif min_distance == float("inf"):
            text = "No neighbours"
        elif min_distance < SAFE_DISTANCE:
            text = f"COLLISION WARNING min={min_distance:.2f}"
        elif min_distance < DETECTION_RADIUS:
            text = f"FLOCKING min={min_distance:.2f}"
        else:
            text = f"SAFE min={min_distance:.2f}"

        self.get_logger().info(
            f"{self.drone_id} ({self.x:.2f}, {self.y:.2f}) {text} "
            f"| goal=({self.goal_x:.1f},{self.goal_y:.1f}) | Bid={self.bid:.3f} | Winner(demo)={winner}"
        )

    def destroy_node(self):
        self.log_file.close()
        self.events_file.close()
        pass_fail = "PASS" if not self.safety_violated else "FAIL (soft threshold)"
        print("\n========== SAFETY SUMMARY ==========")
        print(f"Drone             : {self.drone_id}")
        print(f"Closest distance  : {self.closest_distance_ever:.2f} m")
        print(f"Avoidance events  : {self.avoidance_count}")
        print(f"Soft threshold    : {pass_fail}")
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