import re
import time
import rclpy
import sys
from geometry_msgs.msg import PoseStamped
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from std_msgs.msg import String

class RoboterPosition:

    # Aktuelle Position
    x = 0.0
    y = 0.0
    z = 0.0

    # Startposition
    start_x = 0.0
    start_y = 0.0
    start_z = 0.0

    # Aktorzustände
    greifer_status = "offen"
    sauger_status = "aus"

    moveit_initialized = False

    @classmethod
    def bewege_x(cls, delta):
        cls.x = delta
        cls.y = 0
        cls.z = 0
        #print(f"X: {cls.x}", f"Y: {cls.y}", f"Z: {cls.z}")

    @classmethod
    def bewege_y(cls, delta):
        cls.x = 0
        cls.y = delta
        cls.z = 0
        #print(f"X: {cls.x}", f"Y: {cls.y}", f"Z: {cls.z}")

    @classmethod
    def bewege_z(cls, delta):
        cls.x = 0
        cls.y = 0
        cls.z = delta
        #print(f"X: {cls.x}", f"Y: {cls.y}", f"Z: {cls.z}")

    @classmethod
    def aktuelle_position(cls):
        return (cls.x, cls.y, cls.z)

    @classmethod
    def merke_startposition(cls):
        cls.start_x = cls.x
        cls.start_y = cls.y
        cls.start_z = cls.z
        print(f"Neue Startposition gemerkt: ({cls.start_x}, {cls.start_y}, {cls.start_z})")

    @classmethod
    def gehe_zur_startposition(cls):
        print(f"Zur Startposition zurück: ({cls.start_x}, {cls.start_y}, {cls.start_z})")
        cls.schrittweise_bewegen(cls.start_x, cls.start_y, cls.start_z)

    @classmethod
    def schrittweise_bewegen(cls, ziel_x, ziel_y, ziel_z, schrittweite=1.0):
        cls.aktualisiere_aus_moveit()
        while round(cls.x, 5) != round(ziel_x, 5):
            if cls.x < ziel_x:
                cls.bewege_x(min(schrittweite, ziel_x - cls.x))
            else:
                cls.bewege_x(-min(schrittweite, cls.x - ziel_x))
            time.sleep(0.10)

        while round(cls.y, 5) != round(ziel_y, 5):
            if cls.y < ziel_y:
                cls.bewege_y(min(schrittweite, ziel_y - cls.y))
            else:
                cls.bewege_y(-min(schrittweite, cls.y - ziel_y))
            time.sleep(0.10)

        while round(cls.z, 5) != round(ziel_z, 5):
            if cls.z < ziel_z:
                cls.bewege_z(min(schrittweite, ziel_z - cls.z))
            else:
                cls.bewege_z(-min(schrittweite, cls.z - ziel_z))
            time.sleep(0.10)

        print(f"Neue Position erreicht: {cls.aktuelle_position()}")

    @classmethod
    def greife(cls):
        cls.greifer_status = "geschlossen"
        print("Greifer geschlossen.")

    @classmethod
    def lass_los(cls):
        cls.greifer_status = "offen"
        print("Greifer geöffnet.")

    @classmethod
    def sauger_an(cls):
        cls.sauger_status = "an"
        print("Sauger eingeschaltet.")

    @classmethod
    def sauger_aus(cls):
        cls.sauger_status = "aus"
        print("Sauger ausgeschaltet.")

    @classmethod
    def aktueller_zustand(cls):
        return {
            "befehl": "bewegung",
            "position": cls.aktuelle_position(),
            "greifer": cls.greifer_status,
            "sauger": cls.sauger_status
        }

    # --- 🔧 HIER EINGEFÜGT: Dummy-Methoden für MoveIt ---
    @classmethod
    def init_moveit(cls):
        return
        

    @classmethod
    def aktualisiere_aus_moveit(cls):
        print("")       #von herrn manyak


# --- Bewegungstabelle mit Richtungen ---
BEWEGUNGSBEFEHLE = {
    "gehe nach oben": ("z", 0.10),
    "inkrementiere z": ("z", 0.10),
    "nach oben": ("z", 0.10),
    "gehe hoch": ("z", 0.10),
    "bewege dich hoch": ("z", 0.10),
    "oben": ("z", 0.10),

    "gehe nach unten": ("z", -0.10),
    "unten": ("z", -0.10),
    "dekrementiere z": ("z", -0.10),
    "nach unten": ("z", -0.10),
    "gehe runter": ("z", -0.10),
    "bewege dich runter": ("z", -0.10),

    "gehe nach vorne": ("y", 0.10),
    "vorne": ("y", 0.10),
    "inkrementiere y": ("y", 0.10),
    "nach vorne": ("y", 0.10),
    "gehe vor": ("y", 0.10),
    "bewege dich vor": ("y", 0.10),

    "gehe nach hinten": ("y", -0.10),
    "hinten": ("y", -0.10),
    "dekrementiere y": ("y", -0.10),
    "nach hinten": ("y", -0.10),
    "gehe zurück": ("y", -0.10),
    "bewege dich zurück": ("y", -0.10),

    "gehe nach rechts": ("x", -0.10),
    "rechts": ("x", -0.10),
    "inkrementiere x": ("x", -0.10),
    "nach rechts": ("x", -0.10),
    "gehe rechts": ("x", -0.10),
    "bewege dich nach rechts": ("x", -0.10),

    "gehe nach links": ("x", 0.10),
    "links": ("x", 0.10),
    "dekrementiere x": ("x", 0.10),
    "nach links": ("x", 0.10),
    "gehe links": ("x", 0.10),
    "bewege dich nach links": ("x", 0.10)
}

SONSTIGE_BEFEHLE = {
    "merke neue startposition": "speichere_startposition",
    "merke startposition": "speichere_startposition",
    "zur startposition zurück": "zu_startposition",
    "startposition": "zu_startposition"
    #"greife": lambda: RoboterPosition.greife(),
    #"lass los": lambda: RoboterPosition.lass_los(),
    #"sauger an": lambda: RoboterPosition.sauger_an(),
    #"sauger aus": lambda: RoboterPosition.sauger_aus()
}

def verarbeite_befehl(text):
    text = text.lower().strip()

    match = re.match(r"^(.*?)(?:\s(\d+))?$", text)
    if match:
        befehl = match.group(1).strip()
        print('group: ', match.group(1), ' --- ', match.group(2))
        anzahl = int(match.group(2)) if match.group(2) else 1

        # Bewegungsbefehle
        for key, (achse, richtung) in BEWEGUNGSBEFEHLE.items():
            if befehl.startswith(key):
                print('Befehl erkannt: ', key)

                if achse == "x":
                    RoboterPosition.bewege_x(richtung * anzahl)
                elif achse == "y":
                    RoboterPosition.bewege_y(richtung * anzahl)
                elif achse == "z":
                    RoboterPosition.bewege_z(richtung * anzahl)
                else: 
                    print('ungueltige Achse')
                    continue

                return RoboterPosition.aktueller_zustand()

        # Sonstige Befehle
        for key, argument in SONSTIGE_BEFEHLE.items():
            if befehl.startswith(key):
                print('Befehl erkannt: ', key)
                return {"befehl": argument}

    
    else:
        print("Befehl nicht erkannt:", text)
        return { "befehl": "UNKNOWN_COMMAND" }


if __name__ == "__main__":
    examples = [
        'gehe nach rechts',
        'gehe nach rechts 1',
        'gehe nach rechts',
        'gehe nach links',
        'gehe nach rechts drei',
        'nach rechts',
        'nach links',
        'gehe auf keinen Fall rechts',
        'startposition',
        'merke startposition'
    ] 
    for sample_text in examples:
        print('\n\nInput: ', sample_text)
        cmd = verarbeite_befehl(sample_text)
        print('Befehl:', cmd)

    sys.exit(0)

    while True:
        eingabe = input("Befehl: ")
        result = verarbeite_befehl(eingabe)
        print(result)

