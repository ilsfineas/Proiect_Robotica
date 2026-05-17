"""
Maze Solver - Pioneer P3-DX
CoppeliaSim + Python ZMQ Remote API

Strategie:
- wall following pe peretele drept
- evitare simplă obstacole
- oprire la finish

Versiune stabilă pentru labirinturi simple.
"""

from __future__ import annotations

import math
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# =========================================================
# CONFIG
# =========================================================

START_POS = [-1.075, 0.650, 0.139]
FINISH_POS = [0.725, -0.350]

GOAL_TOLERANCE = 0.15

RESET_ROBOT_TO_START = True

# =========================================================
# ROBOT PATHS
# =========================================================

ROBOT_PATH = "/PioneerP3DX"

LEFT_MOTOR_PATH = "/PioneerP3DX/leftMotor"
RIGHT_MOTOR_PATH = "/PioneerP3DX/rightMotor"

SENSOR_PATH_TEMPLATE = "/PioneerP3DX/ultrasonicSensor[{index}]"

# =========================================================
# SENSOR GROUPS
# =========================================================

# conform documentatie laborator
FRONT_SENSORS = [3, 4]

RIGHT_SENSORS = [8, 9]

# =========================================================
# CONTROL PARAMETERS
# =========================================================

SENSOR_MAX = 1.0

V_BASE = 2.0

TARGET_DIST = 0.35

K_P = 3.0

FRONT_STOP = 0.30

MAX_SPEED = 4.0

STATUS_PERIOD = 0.5

# =========================================================
# HELPERS
# =========================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def euclidean_2d(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def read_min_dist(sim, sensors, indices):

    min_dist = SENSOR_MAX

    for idx in indices:

        result, dist, *_ = sim.readProximitySensor(sensors[idx])

        if result and dist < min_dist:
            min_dist = dist

    return min_dist


def set_velocity(sim, left_motor, right_motor, v_left, v_right):

    v_left = clamp(v_left, -MAX_SPEED, MAX_SPEED)
    v_right = clamp(v_right, -MAX_SPEED, MAX_SPEED)

    sim.setJointTargetVelocity(left_motor, v_left)
    sim.setJointTargetVelocity(right_motor, v_right)


# =========================================================
# MAIN
# =========================================================

def main():

    print("Conectare la CoppeliaSim...")

    client = RemoteAPIClient()
    sim = client.require("sim")

    # -----------------------------------------------------

    robot = sim.getObject(ROBOT_PATH)

    left_motor = sim.getObject(LEFT_MOTOR_PATH)
    right_motor = sim.getObject(RIGHT_MOTOR_PATH)

    sensors = [
        sim.getObject(
            SENSOR_PATH_TEMPLATE.format(index=i)
        )
        for i in range(16)
    ]

    # -----------------------------------------------------

    if RESET_ROBOT_TO_START:

        sim.setObjectPosition(
            robot,
            sim.handle_world,
            START_POS
        )

    # -----------------------------------------------------

    sim.startSimulation()

    print("Simulare pornita.")
    print("Wall following pe peretele drept.")
    print()

    last_status_time = -1.0

    try:

        while True:

            current_time = sim.getSimulationTime()

            # =================================================
            # POZITIE ROBOT
            # =================================================

            pos = sim.getObjectPosition(
                robot,
                sim.handle_world
            )

            goal_distance = euclidean_2d(
                pos,
                FINISH_POS
            )

            # =================================================
            # FINISH
            # =================================================

            if goal_distance <= GOAL_TOLERANCE:

                set_velocity(
                    sim,
                    left_motor,
                    right_motor,
                    0.0,
                    0.0
                )

                print()
                print("===================================")
                print("FINISH GASIT")
                print(f"x = {pos[0]:+.3f}")
                print(f"y = {pos[1]:+.3f}")
                print("===================================")

                break

            # =================================================
            # SENZORI
            # =================================================

            dist_front = read_min_dist(
                sim,
                sensors,
                FRONT_SENSORS
            )

            dist_right = read_min_dist(
                sim,
                sensors,
                RIGHT_SENSORS
            )

            # =================================================
            # LOGICA CONTROL
            # =================================================

            # -------------------------------------------------
            # obstacol frontal
            # -------------------------------------------------

            if dist_front < FRONT_STOP:

                v_left = -V_BASE
                v_right = +V_BASE

                state = (
                    f"TURN LEFT  "
                    f"front={dist_front:.3f}"
                )

            # -------------------------------------------------
            # nu exista perete la dreapta
            # -------------------------------------------------

            elif dist_right >= SENSOR_MAX * 0.95:

                v_left = V_BASE
                v_right = V_BASE * 0.65

                state = "SEARCH WALL"

            # -------------------------------------------------
            # wall following
            # -------------------------------------------------

            else:

                error = dist_right - TARGET_DIST

                v_left = V_BASE + K_P * error
                v_right = V_BASE - K_P * error

                state = (
                    f"FOLLOW  "
                    f"right={dist_right:.3f} "
                    f"err={error:+.3f}"
                )

            # =================================================
            # LIMITARE VITEZE
            # =================================================

            v_left = clamp(v_left, -MAX_SPEED, MAX_SPEED)
            v_right = clamp(v_right, -MAX_SPEED, MAX_SPEED)

            # =================================================
            # COMANDA MOTOARE
            # =================================================

            set_velocity(
                sim,
                left_motor,
                right_motor,
                v_left,
                v_right
            )

            # =================================================
            # DEBUG
            # =================================================

            if current_time - last_status_time >= STATUS_PERIOD:

                print(
                    f"[{state:<30}] "
                    f"goal={goal_distance:.3f} "
                    f"front={dist_front:.3f} "
                    f"right={dist_right:.3f} "
                    f"vL={v_left:+.2f} "
                    f"vR={v_right:+.2f}"
                )

                last_status_time = current_time

            # =================================================

            sensor_values = []

            for i in range(16):
                result, dist, *_ = sim.readProximitySensor(sensors[i])
                if result:
                    sensor_values.append(round(dist, 3))
                else:
                    sensor_values.append(None)
            print(sensor_values)

            time.sleep(0.05)
            for i, d in enumerate(sensor_values):
                print(f"S{i}: {d}")

    except KeyboardInterrupt:

        print("\nProgram intrerupt manual.")

    finally:

        set_velocity(
            sim,
            left_motor,
            right_motor,
            0.0,
            0.0
        )

        sim.stopSimulation()

        print("Simulare oprita.")


# =========================================================

if __name__ == "__main__":
    main()