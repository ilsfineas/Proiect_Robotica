import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# =========================================================
# CONFIG
# =========================================================

START_POS = [8.425, 10.925, 0.139]
FINISH_POS = [-0.550, 3.375]

FORWARD_SPEED = 1.2
TURN_GAIN = 0.8

WALL_DISTANCE = 0.45
FRONT_THRESHOLD = 0.6

TARGET_RADIUS = 0.3

# =========================================================
# SENSOR GROUPS
# =========================================================

FRONT_SENSORS = [3, 4]
LEFT_SENSORS = [0, 1, 14, 15]
RIGHT_SENSORS = [6, 7, 8, 9]

# =========================================================
# ROBOT SETUP
# =========================================================

def ensure_pioneer(sim):

    robot = sim.getObject('/PioneerP3DX')

    left_motor = sim.getObject('/PioneerP3DX/leftMotor')
    right_motor = sim.getObject('/PioneerP3DX/rightMotor')

    sensors = [
        sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
        for i in range(16)
    ]

    return robot, left_motor, right_motor, sensors

# =========================================================
# SENSOR HELPERS
# =========================================================

def get_min_distance(sim, sensors, indices):

    min_dist = 999

    for idx in indices:

        result, dist, *_ = sim.readProximitySensor(
            sensors[idx]
        )

        if result:
            min_dist = min(min_dist, dist)

    return min_dist

# =========================================================
# TARGET CHECK
# =========================================================

def reached_goal(pos):

    dx = FINISH_POS[0] - pos[0]
    dy = FINISH_POS[1] - pos[1]

    dist = math.sqrt(dx * dx + dy * dy)

    return dist < TARGET_RADIUS

# =========================================================
# M-LINE CHECK
# linia START -> FINISH
# =========================================================

def distance_to_mline(x, y):

    x1 = START_POS[0]
    y1 = START_POS[1]

    x2 = FINISH_POS[0]
    y2 = FINISH_POS[1]

    numerator = abs(
        (y2 - y1) * x
        - (x2 - x1) * y
        + x2 * y1
        - y2 * x1
    )

    denominator = math.sqrt(
        (y2 - y1) ** 2
        + (x2 - x1) ** 2
    )

    return numerator / denominator

# =========================================================
# GO TO GOAL
# =========================================================

def go_to_goal(sim, robot, left_motor, right_motor):

    pos = sim.getObjectPosition(
        robot,
        sim.handle_world
    )

    ori = sim.getObjectOrientation(
        robot,
        sim.handle_world
    )

    rx = pos[0]
    ry = pos[1]

    yaw = ori[2]

    dx = FINISH_POS[0] - rx
    dy = FINISH_POS[1] - ry

    desired_angle = math.atan2(dy, dx)

    angle_error = desired_angle - yaw

    while angle_error > math.pi:
        angle_error -= 2 * math.pi

    while angle_error < -math.pi:
        angle_error += 2 * math.pi

    angular = TURN_GAIN * angle_error

    left_speed = FORWARD_SPEED - angular
    right_speed = FORWARD_SPEED + angular

    left_speed = max(min(left_speed, 3), -3)
    right_speed = max(min(right_speed, 3), -3)

    sim.setJointTargetVelocity(
        left_motor,
        left_speed
    )

    sim.setJointTargetVelocity(
        right_motor,
        right_speed
    )

# =========================================================
# WALL FOLLOWING
# =========================================================

def follow_wall(
    sim,
    sensors,
    left_motor,
    right_motor
):

    d_front = get_min_distance(
        sim,
        sensors,
        FRONT_SENSORS
    )

    d_left = get_min_distance(
        sim,
        sensors,
        LEFT_SENSORS
    )

    # obstacle directly ahead
    if d_front < FRONT_THRESHOLD:

        left_speed = 1.5
        right_speed = -1.0

    else:

        # maintain wall distance
        error = WALL_DISTANCE - d_left

        correction = 3.0 * error

        left_speed = 1.8 - correction
        right_speed = 1.8 + correction

    left_speed = max(min(left_speed, 3), -3)
    right_speed = max(min(right_speed, 3), -3)

    sim.setJointTargetVelocity(
        left_motor,
        left_speed
    )

    sim.setJointTargetVelocity(
        right_motor,
        right_speed
    )

# =========================================================
# MAIN
# =========================================================

def main():

    client = RemoteAPIClient()

    sim = client.require('sim')

    robot, left_motor, right_motor, sensors = ensure_pioneer(sim)

    sim.startSimulation()

    try:

        print("=== BUG2 MAZE SOLVER START ===")

        sim.setObjectPosition(
            robot,
            sim.handle_world,
            START_POS
        )

        sim.setObjectOrientation(
            robot,
            sim.handle_world,
            [0, 0, 0]
        )

        time.sleep(1)

        mode = "GOAL"

        hit_point_distance = None

        while True:

            pos = sim.getObjectPosition(
                robot,
                sim.handle_world
            )

            rx = pos[0]
            ry = pos[1]

            # =============================================
            # SUCCESS
            # =============================================

            if reached_goal(pos):

                print("=== GOAL REACHED ===")

                break

            # =============================================
            # SENSOR READ
            # =============================================

            d_front = get_min_distance(
                sim,
                sensors,
                FRONT_SENSORS
            )

            # =============================================
            # GO TO GOAL MODE
            # =============================================

            if mode == "GOAL":

                if d_front < FRONT_THRESHOLD:

                    print("Obstacle hit -> WALL mode")

                    mode = "WALL"

                    dx = FINISH_POS[0] - rx
                    dy = FINISH_POS[1] - ry

                    hit_point_distance = math.sqrt(
                        dx * dx + dy * dy
                    )

                else:

                    go_to_goal(
                        sim,
                        robot,
                        left_motor,
                        right_motor
                    )

            # =============================================
            # WALL FOLLOW MODE
            # =============================================

            else:

                follow_wall(
                    sim,
                    sensors,
                    left_motor,
                    right_motor
                )

                # near m-line again?
                mline_dist = distance_to_mline(rx, ry)

                dx = FINISH_POS[0] - rx
                dy = FINISH_POS[1] - ry

                current_goal_dist = math.sqrt(
                    dx * dx + dy * dy
                )

                # leave obstacle
                if (
                    mline_dist < 0.2
                    and current_goal_dist < hit_point_distance
                ):

                    print("Back to GOAL mode")

                    mode = "GOAL"

            time.sleep(0.05)

    except KeyboardInterrupt:

        print("STOPPED")

    finally:

        sim.setJointTargetVelocity(left_motor, 0)
        sim.setJointTargetVelocity(right_motor, 0)

        sim.stopSimulation()

# =========================================================

if __name__ == "__main__":

    main()