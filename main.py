import time
import math
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# --- PARAMETRI GLOBALI ---
START_POS = [+8.425, +10.925, 0.139]
FINISH_POS = [-0.550, +3.375]  
FINISH_RADIUS = 0.25 # Distanta (metri) considerata "succes" fata de centrul patratului negru

V_FORWARD = 2.0
V_TURN = 1.5
WALL_THRESH = 0.4    # Distanta la care consideram ca exista un zid
CELL_SIZE = 0.5      # Dimensiunea grilei pentru rotunjirea coordonatelor in memorie

# Memoria globala a "robotilor" anteriori
dead_ends = set()

# Gruparea senzorilor
FRONT_SENSORS = [3, 4]       
LEFT_SENSORS = [0, 1, 14, 15]
RIGHT_SENSORS = [6, 7, 8, 9] 


def ensure_pioneer_model(sim):
    """Returneaza handle-urile robotului, incarcand modelul daca lipseste din scena."""
    try:
        robot = sim.getObject('/PioneerP3DX')
    except Exception:
        model_path = Path(__file__).resolve().parent / 'models' / 'PioneerP3DX.ttm'
        sim.loadModel(str(model_path))
        robot = sim.getObject('/PioneerP3DX')

    left_motor = sim.getObject('/PioneerP3DX/leftMotor')
    right_motor = sim.getObject('/PioneerP3DX/rightMotor')
    sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]') for i in range(16)]
    return robot, left_motor, right_motor, sensors

def discretize_pos(pos):
    """Transforma o coordonata continua intr-un nod de grila pentru memorie."""
    return (round(pos[0] / CELL_SIZE), round(pos[1] / CELL_SIZE))

def get_min_dist(sim, sensors, indices):
    """Returneaza distanta minima pentru un grup de senzori."""
    min_dist = 1.0
    for idx in indices:
        result, dist, *_ = sim.readProximitySensor(sensors[idx])
        if result and dist < min_dist:
            min_dist = dist
    return min_dist

def spawn_new_robot(sim, robot, left_motor, right_motor):
    """Opreste motoarele si teleporteaza robotul inapoi la start (Noua iteratie)."""
    print("\n--- SPAWN ROBOT NOU LA START ---")
    sim.setJointTargetVelocity(left_motor, 0.0)
    sim.setJointTargetVelocity(right_motor, 0.0)
    time.sleep(0.5)
    
    # Teleportare la start
    sim.setObjectPosition(robot, sim.handle_world, START_POS)
    # Resetare orientare (cu fata in directia initiala)
    sim.setObjectOrientation(robot, sim.handle_world, [0, 0, 0])
    time.sleep(1.0) # Pauza de stabilizare fizica

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    # Obtinere handle-uri; daca modelul nu exista in scena, il incarcam din workspace.
    robot, left_motor, right_motor, sensors = ensure_pioneer_model(sim)

    sim.startSimulation()
    print("=== START EXPLORARE LABIRINT ===")
    
    # Asigura-te ca incepem de la punctul de start setat de tine
    spawn_new_robot(sim, robot, left_motor, right_motor)

    iteration = 1
    
    try:
        while True:
            # 1. VERIFICARE SUCCES (A ajuns la patratul negru?)
            pos = sim.getObjectPosition(robot, sim.handle_world)
            dist_to_finish = math.dist([pos[0], pos[1]], FINISH_POS)
            
            if dist_to_finish < FINISH_RADIUS:
                print(f"\n[SUCCES] Labirint rezolvat din {iteration} iteratii!")
                sim.setJointTargetVelocity(left_motor, 0.0)
                sim.setJointTargetVelocity(right_motor, 0.0)
                break

            # 2. CITIRE SENZORI
            d_front = get_min_dist(sim, sensors, FRONT_SENSORS)
            d_left = get_min_dist(sim, sensors, LEFT_SENSORS)
            d_right = get_min_dist(sim, sensors, RIGHT_SENSORS)

            # 3. VERIFICARE INFUNDATURA (DEAD END)
            if d_front < WALL_THRESH and d_left < WALL_THRESH and d_right < WALL_THRESH:
                current_grid = discretize_pos(pos)
                print(f"[INFUNDATURA] Detectata la grila {current_grid}. Memorez...")
                dead_ends.add(current_grid)
                
                iteration += 1
                spawn_new_robot(sim, robot, left_motor, right_motor)
                continue

            # 4. LOGICA DE NAVIGARE SI EVITARE MEMORATA
            # Daca drumul in fata e blocat, trebuie sa alegem o directie
            if d_front < WALL_THRESH:
                # Estimam coordonatele grilei din stanga si din dreapta
                # (O euristica simpla: adunam un offset la coordonatele curente bazat pe orientare)
                ori = sim.getObjectOrientation(robot, sim.handle_world)
                yaw = ori[2]
                
                grid_left = discretize_pos([pos[0] - math.sin(yaw)*0.5, pos[1] + math.cos(yaw)*0.5])
                grid_right = discretize_pos([pos[0] + math.sin(yaw)*0.5, pos[1] - math.cos(yaw)*0.5])

                # Verificam memoria
                if grid_left in dead_ends:
                    print("Stanga e marcata ca infundatura! Virez dreapta.")
                    v_left, v_right = V_TURN, -V_TURN
                elif grid_right in dead_ends:
                    print("Dreapta e marcata ca infundatura! Virez stanga.")
                    v_left, v_right = -V_TURN, V_TURN
                else:
                    # Niciuna nu e cunoscuta ca infundatura, alegem calea mai libera
                    if d_left > d_right:
                        v_left, v_right = -V_TURN, V_TURN
                    else:
                        v_left, v_right = V_TURN, -V_TURN
            else:
                # Mers inainte cu un usor wall-following pentru a merge drept
                error = d_right - d_left
                v_left = V_FORWARD - 0.5 * error
                v_right = V_FORWARD + 0.5 * error

            # Aplicare viteze
            sim.setJointTargetVelocity(left_motor, v_left)
            sim.setJointTargetVelocity(right_motor, v_right)

            time.sleep(0.05) # Bucla de 20Hz

    except KeyboardInterrupt:
        print("\nExplorare oprita manual.")
    finally:
        # Oprire garantata
        sim.setJointTargetVelocity(left_motor, 0.0)
        sim.setJointTargetVelocity(right_motor, 0.0)
        sim.stopSimulation()

if __name__ == '__main__':
    main()