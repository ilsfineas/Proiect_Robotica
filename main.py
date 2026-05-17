import time
import math
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# --- Configurații Labirint și Roi ---
START_POS  = [0.975, 1.000, 0.139] # Coordonatele tale de start (Z-ul este înălțimea)
FINISH_POS = [-0.745, -0.225]      # Coordonatele tale de final (X, Y)
FINISH_RADIUS = 0.4                # Marja de eroare pentru finish

# Modelul este stocat în folderul local models/
MODEL_PATH = (Path(__file__).resolve().parent / "models" / "PioneerP3DX.ttm").as_posix()

V_FORWARD = 2.0         # rad/s - viteza de deplasare înainte
V_TURN = 2.0            # rad/s - viteza de viraj
TIME_CELL = 2.5         # Timp pentru a avansa o "celulă"
TIME_TURN_90 = 1.57     # Timp pentru un viraj de ~90 grade
THRESHOLD_WALL = 0.6    # Pragul sub care un senzor consideră că e un obstacol

# Mapping senzori în grupuri (acoperire circulară). Ajustează dacă modelul
# are o distribuirre diferită a senzorilor.
SENSOR_GROUPS = {
    'Stanga': [11,12,13,14,15,0],
    'Fata':   [1,2,3,4,5],
    'Dreapta':[6,7,8,9,10]
}


class RobotInstance:
    def __init__(self, sim, robot_handle):
        self.sim = sim
        self.handle = robot_handle
        
        # Obținem alias-ul generat automat (ex: PioneerP3DX#0) pentru a-i găsi corect copiii
        alias = self.sim.getObjectAlias(self.handle)
        
        # Căutăm motoarele și senzorii specifici acestei instanțe
        self.left_motor = self.sim.getObject(f'/{alias}/leftMotor')
        self.right_motor = self.sim.getObject(f'/{alias}/rightMotor')
        
        self.sensors = [
            self.sim.getObject(f'/{alias}/ultrasonicSensor[{i}]') for i in range(16)
        ]

    def set_motors(self, v_left, v_right):
        self.sim.setJointTargetVelocity(self.left_motor, v_left)
        self.sim.setJointTargetVelocity(self.right_motor, v_right)

    def stop(self):
        self.set_motors(0.0, 0.0)
        time.sleep(0.5)

    def move_forward(self):
        self.set_motors(V_FORWARD, V_FORWARD)
        time.sleep(TIME_CELL)
        self.stop()

    def turn(self, direction):
        if direction == 'Dreapta':
            self.set_motors(V_TURN, -V_TURN)
        elif direction == 'Stanga':
            self.set_motors(-V_TURN, V_TURN)
        elif direction == 'Inapoi': 
            self.set_motors(V_TURN, -V_TURN)
            time.sleep(TIME_TURN_90 * 2)
            self.stop()
            return
            
        time.sleep(TIME_TURN_90)
        self.stop()

    def get_min_distance(self, indices):
        """Returnează distanța minimă detectată de un grup de senzori."""
        min_dist = 1.0 # Considerăm 1.0m ca fiind drum liber maxim
        for idx in indices:
            res, dist, *_ = self.sim.readProximitySensor(self.sensors[idx])
            if res and dist < min_dist:
                min_dist = dist
        return min_dist

    def scan_intersections(self):
        # Citim toți senzorii pentru debugging detaliat (utile pentru mapare)
        alias = self.sim.getObjectAlias(self.handle)
        per_sensor = []
        for idx in range(len(self.sensors)):
            res, dist, *_ = self.sim.readProximitySensor(self.sensors[idx])
            if not res:
                dist = 1.0
            per_sensor.append((idx, dist))
        # Afișăm citirile (scurt)
        per_str = ', '.join([f"{i}:{d:.2f}" for i,d in per_sensor])
        print(f"[{alias}] senzori -> {per_str}")

        # Calculăm distanțele minime pe grupuri definite mai sus
        dist_l = self.get_min_distance(SENSOR_GROUPS['Stanga'])
        dist_f = self.get_min_distance(SENSOR_GROUPS['Fata'])
        dist_r = self.get_min_distance(SENSOR_GROUPS['Dreapta'])
        print(f"[{alias}] Vede -> Stanga: {dist_l:.2f}m | Fata: {dist_f:.2f}m | Dreapta: {dist_r:.2f}m")

        available_directions = []
        # Regula priorității: Dreapta -> Înainte -> Stânga
        if dist_r > THRESHOLD_WALL:
            available_directions.append('Dreapta')
        if dist_f > THRESHOLD_WALL:
            available_directions.append('Inainte')
        if dist_l > THRESHOLD_WALL:
            available_directions.append('Stanga')

        return available_directions

    def is_at_finish(self):
        pos = self.sim.getObjectPosition(self.handle, self.sim.handle_world)
        dist = math.sqrt((pos[0] - FINISH_POS[0])**2 + (pos[1] - FINISH_POS[1])**2)
        return dist < FINISH_RADIUS

    def run_until_stuck_or_finish(self):
        while True:
            if self.is_at_finish():
                return "FINISH"

            options = self.scan_intersections()

            if len(options) == 0:
                self.stop()
                return "INFUNDATURA" # Rămâne pe loc și devine perete

            alias = self.sim.getObjectAlias(self.handle)
            # Încercăm fiecare opțiune în ordinea priorității, dar testăm sigur
            moved = False
            for chosen_direction in options:
                print(f"[{alias}] Încearcă: {chosen_direction}")
                # executăm virajul temporar (dacă este cazul)
                turned = False
                if chosen_direction != 'Inainte':
                    self.turn(chosen_direction)
                    turned = True

                # după viraj, verificăm dacă fața (grupul front) e liberă
                front_clear = self.get_min_distance(SENSOR_GROUPS['Fata']) > THRESHOLD_WALL
                print(f"[{alias}] Front clear after turn: {front_clear}")

                if front_clear:
                    # putem merge înainte
                    self.move_forward()
                    moved = True
                    break
                else:
                    # dacă am întors și nu e liber, revenim la orientarea anterioară
                    if turned:
                        # inversăm virajul: Dreapta <-> Stanga
                        if chosen_direction == 'Dreapta':
                            self.turn('Stanga')
                        elif chosen_direction == 'Stanga':
                            self.turn('Dreapta')
                        else:
                            # pentru orice alt caz (safety)
                            self.turn('Inapoi')
                    # continuăm la următoarea opțiune

            if not moved:
                # Niciuna din opțiuni nu a permis un pas sigur
                self.stop()
                return "INFUNDATURA"


def main():
    if not Path(MODEL_PATH).exists():
        raise FileNotFoundError(f"Modelul nu a fost găsit: {MODEL_PATH}")

    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.startSimulation()
    print("Simularea a început. Generăm roboți până rezolvăm labirintul...")
    time.sleep(1)

    max_robots = 50 
    
    try:
        for i in range(max_robots):
            print(f"\n--- Spawnare Robot #{i+1} la Start ---")
            
            robot_handle = sim.loadModel(MODEL_PATH)
            
            sim.setObjectPosition(robot_handle, sim.handle_world, START_POS)
            sim.setObjectOrientation(robot_handle, sim.handle_world, [0, 0, 0])
            time.sleep(0.5) 

            robot = RobotInstance(sim, robot_handle)
            rezultat = robot.run_until_stuck_or_finish()

            if rezultat == "FINISH":
                print(f"🎉 SUCCES! Robotul #{i+1} a găsit calea către Finish!")
                break
            else:
                print(f"❌ Robotul #{i+1} a ajuns într-o înfundătură și va bloca drumul.")
                
    except KeyboardInterrupt:
        print("\nOprit manual din consolă.")
    finally:
        sim.stopSimulation()
        print("Simularea s-a oprit.")

if __name__ == '__main__':
    main()