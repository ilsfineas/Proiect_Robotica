# ============================================================
#  CoppeliaSim Maze Solver (Remote API)
#  Improved Left-Wall Follower with:
#   - loop detection
#   - progress watchdog
#   - aggressive recovery
#   - corridor centering
#   - PID stabilization
# ============================================================

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import time

# ============================================================
# CONFIG
# ============================================================

START_POS = [8.425, 10.925, 0.139]
FINISH_POS = [-0.550, 3.375]

GOAL_TOLERANCE = 0.45

TARGET_LEFT_DIST = 0.35

FRONT_BRAKE_DIST = 0.28
FRONT_SLOW_DIST = 0.45

BASE_SPEED = 2.0

KP = 0.7
KI = 0.0
KD = 0.15

# ============================================================
# MAZE SOLVER
# ============================================================

class MazeSolver:

    def __init__(self):

        client = RemoteAPIClient()
        self.sim = client.require('sim')

        # ----------------------------------------------------
        # OBJECTS
        # ----------------------------------------------------

        self.robot = self.sim.getObject('/PioneerP3DX')

        self.left_motor = self.sim.getObject(
            '/PioneerP3DX/leftMotor'
        )

        self.right_motor = self.sim.getObject(
            '/PioneerP3DX/rightMotor'
        )

        # ultrasonic sensors
        self.sensors = []

        for i in range(16):
            sensor = self.sim.getObject(
                f'/PioneerP3DX/ultrasonicSensor[{i}]'
            )
            self.sensors.append(sensor)

        # ----------------------------------------------------
        # PID
        # ----------------------------------------------------

        self.integral = 0.0
        self.prev_error = 0.0

        # ----------------------------------------------------
        # LOOP DETECTION
        # ----------------------------------------------------

        self.visited = set()

        # ----------------------------------------------------
        # PROGRESS WATCHDOG
        # ----------------------------------------------------

        self.best_distance = 9999
        self.last_progress_time = time.time()

    # ========================================================
    # HELPERS
    # ========================================================

    def set_velocity(self, left, right):

        self.sim.setJointTargetVelocity(
            self.left_motor,
            left
        )

        self.sim.setJointTargetVelocity(
            self.right_motor,
            right
        )

    # --------------------------------------------------------

    def get_position(self):

        pos = self.sim.getObjectPosition(
            self.robot,
            -1
        )

        return pos

    # --------------------------------------------------------

    def distance_to_goal(self, pos):

        dx = FINISH_POS[0] - pos[0]
        dy = FINISH_POS[1] - pos[1]

        return math.sqrt(dx * dx + dy * dy)

    # --------------------------------------------------------

    def get_cell(self, pos, cell_size=0.5):

        return (
            round(pos[0] / cell_size),
            round(pos[1] / cell_size)
        )

    # --------------------------------------------------------

    def read_sensors(self):

        dists = []

        for sensor in self.sensors:

            result, distance, _, _, _ = (
                self.sim.readProximitySensor(sensor)
            )

            if result:
                dists.append(distance)
            else:
                dists.append(0.8)

        return dists

    # --------------------------------------------------------

    def compute_left_distance(self, dists):

        left_indices = [0, 1, 2, 3]

        vals = [dists[i] for i in left_indices]

        return min(vals)

    # ========================================================
    # CONTROL LOOP
    # ========================================================

    def solve(self):

        print("\n==============================")
        print(" STARTING MAZE SOLVER ")
        print("==============================\n")

        self.sim.startSimulation()

        time.sleep(1.0)

        try:

            while True:

                # ============================================
                # POSITION
                # ============================================

                pos = self.get_position()

                dist_finish = self.distance_to_goal(pos)

                print(
                    f"[POS] x={pos[0]:.3f} "
                    f"y={pos[1]:.3f} "
                    f"dist_finish={dist_finish:.3f}"
                )

                # ============================================
                # GOAL REACHED
                # ============================================

                if dist_finish < GOAL_TOLERANCE:

                    print("\n================================")
                    print(" MAZE SOLVED!")
                    print("================================\n")

                    self.set_velocity(0, 0)
                    break

                # ============================================
                # LOOP DETECTION
                # ============================================

                cell = self.get_cell(pos)

                repeated = cell in self.visited

                if not repeated:
                    self.visited.add(cell)

                # ============================================
                # PROGRESS WATCHDOG
                # ============================================

                if dist_finish < self.best_distance:

                    self.best_distance = dist_finish
                    self.last_progress_time = time.time()

                stuck_timeout = 12.0

                if (
                    time.time() - self.last_progress_time
                    > stuck_timeout
                ):

                    print("\n[RECOVERY] STUCK")
                    print("Executing escape maneuver...\n")

                    # reverse
                    self.set_velocity(-2.0, -2.0)
                    time.sleep(1.2)

                    # strong left turn
                    self.set_velocity(-2.5, 2.5)
                    time.sleep(2.2)

                    self.last_progress_time = time.time()

                    continue

                # ============================================
                # SENSOR READ
                # ============================================

                dists = self.read_sensors()

                # front sensors
                front_indices = [3, 4, 5]

                front_min = min(
                    [dists[i] for i in front_indices]
                )

                # ============================================
                # WALL FOLLOW
                # ============================================

                left_dist = min(
                    self.compute_left_distance(dists),
                    0.8
                )

                error = TARGET_LEFT_DIST - left_dist

                self.integral += error

                derivative = error - self.prev_error

                self.prev_error = error

                steer = (
                    KP * error
                    + KI * self.integral
                    + KD * derivative
                )

                # ============================================
                # CORRIDOR CENTERING
                # ============================================

                front_left = min(
                    dists[1],
                    dists[2],
                    dists[3]
                )

                front_right = min(
                    dists[4],
                    dists[5],
                    dists[6]
                )

                corridor_error = (
                    front_right - front_left
                )

                steer += corridor_error * 0.4

                # ============================================
                # BEHAVIOR LOGIC
                # ============================================

                state = "FOLLOW"

                # HARD TURN
                if front_min < FRONT_BRAKE_DIST:

                    v_left = -2.5
                    v_right = 2.5

                    state = "TURN"

                # SLOW TURN
                elif front_min < FRONT_SLOW_DIST:

                    v_left = 0.3
                    v_right = 1.8

                    state = "SLOW"

                # NORMAL FOLLOW
                else:

                    v_left = BASE_SPEED + steer
                    v_right = BASE_SPEED - steer

                # ============================================
                # LOOP ESCAPE
                # ============================================

                if repeated:

                    print("[LOOP] Revisited cell")

                    v_left -= 0.8
                    v_right += 0.8

                # ============================================
                # CLAMP SPEEDS
                # ============================================

                v_left = max(min(v_left, 3.0), -3.0)
                v_right = max(min(v_right, 3.0), -3.0)

                # ============================================
                # DEBUG
                # ============================================

                print(
                    f"[{state:^7}] "
                    f"left={left_dist:.3f} "
                    f"front={front_min:.3f} "
                    f"err={error:.3f} "
                    f"vL={v_left:+.2f} "
                    f"vR={v_right:+.2f}"
                )

                # ============================================
                # APPLY VELOCITIES
                # ============================================

                self.set_velocity(v_left, v_right)

                time.sleep(0.05)

        except KeyboardInterrupt:

            print("\nInterrupted by user")

        finally:

            self.set_velocity(0, 0)

            time.sleep(0.2)

            self.sim.stopSimulation()

            print("\nSimulation stopped")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    solver = MazeSolver()

    solver.solve()