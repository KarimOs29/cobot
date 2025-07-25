import re
import time



class RoboterPosition:
    """
    Klasse zur Verwaltung der Roboterposition und Aktor-Zustände.
    """

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

    # --- Positionsmethoden ---

    @classmethod
    def bewege_x(cls, delta):
        cls.x += delta
        print(f"X: {cls.x}", f"Y: {cls.y}",f"Z: {cls.z}")

    @classmethod
    def bewege_y(cls, delta):
        cls.y += delta
        print(f"X: {cls.x}", f"Y: {cls.y}",f"Z: {cls.z}")

    @classmethod
    def bewege_z(cls, delta):
        cls.z += delta
        print(f"X: {cls.x}", f"Y: {cls.y}",f"Z: {cls.z}")

    @classmethod
    def aktuelle_position(cls):
        return (cls.x, cls.y, cls.z)

    @classmethod
    def merke_startposition(cls):
        cls.start_x = cls.x
        cls.start_y = cls.y
        cls.start_z = cls.z
        print(f"Neue Startposition gemerkt: ({cls.start_x}, {cls.start_y},{cls.start_z})")








    @classmethod
    def gehe_zur_startposition(cls):
        print(f"Zur Startposition zurück: ({cls.start_x}, {cls.start_y}, {cls.start_z})")
        cls.schrittweise_bewegen(cls.start_x, cls.start_y, cls.start_z)

    @classmethod
    def schrittweise_bewegen(cls, ziel_x, ziel_y, ziel_z, schrittweite=1.0):
        """
        Bewegt den Roboter schrittweise zu einer Zielposition.
        """
        # X-Achse
        while round(cls.x, 5) != round(ziel_x, 5):
            if cls.x < ziel_x:
                cls.bewege_x(min(schrittweite, ziel_x - cls.x))
            else:
                cls.bewege_x(-min(schrittweite, cls.x - ziel_x))
            time.sleep(0.05)

        # Y-Achse
        while round(cls.y, 5) != round(ziel_y, 5):
            if cls.y < ziel_y:
                cls.bewege_y(min(schrittweite, ziel_y - cls.y))
            else:
                cls.bewege_y(-min(schrittweite, cls.y - ziel_y))
            time.sleep(0.05)

        # Z-Achse
        while round(cls.z, 5) != round(ziel_z, 5):
            if cls.z < ziel_z:
                cls.bewege_z(min(schrittweite, ziel_z - cls.z))
            else:
                cls.bewege_z(-min(schrittweite, cls.z - ziel_z))
            time.sleep(0.05)

        print(f"Neue Position erreicht: {cls.aktuelle_position()}")

    # --- Aktor-Methoden ---


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

    # --- Zustandsabfrage ---

    @classmethod
    def aktueller_zustand(cls):
        """
        Gibt den aktuellen Zustand des Roboters zurück (Position + Aktoren).
        """
        return {
            "position": (cls.x, cls.y, cls.z),
            "greifer": cls.greifer_status,
            "sauger": cls.sauger_status
        }

# --- Bewegungstabelle mit Richtungen ---

# Hier wird festgelegt, wie sich jeder Befehl auf X, Y oder Z auswirkt
BEWEGUNGSBEFEHLE = {
    "gehe nach oben": ("y", 1),
    "inkrementiere y": ("y", 1),
    "nach oben": ("y", 1),
    "gehe hoch": ("y", 1),
    "bewege dich hoch": ("y", 1),

    "gehe nach unten": ("y", -1),
    "dekrementiere y": ("y", -1),
    "nach unten": ("y", -1),
    "gehe runter": ("y", -1),
    "bewege dich runter": ("y", -1),

    "gehe nach vorne": ("z", 1),
    "inkrementiere z": ("z", 1),
    "nach vorne": ("z", 1),
    "gehe vor": ("z", 1),
    "bewege dich vor": ("z", 1),

    "gehe nach hinten": ("z", -1),
    "dekrementiere z": ("z", -1),
    "nach hinten": ("z", -1),
    "gehe zurück": ("z", -1),
    "bewege dich zurück": ("z", -1),

    "gehe nach rechts": ("x", 1),
    "inkrementiere x": ("x", 1),
    "nach rechts": ("x", 1),
    "gehe rechts": ("x", 1),
    "bewege dich nach rechts": ("x", 1),

    "gehe nach links": ("x", -1),
    "dekrementiere x": ("x", -1),
    "nach links": ("x", -1),
    "gehe links": ("x", -1),
    "bewege dich nach links": ("x", -1)
}

# --- Sonstige Befehle (Greifer, Sauger, Startposition) ---

SONSTIGE_BEFEHLE = {
    "merke neue startposition": lambda
: RoboterPosition.merke_startposition(),
    "zur startposition zurück": lambda
: RoboterPosition.gehe_zur_startposition(),
    "greife": lambda: RoboterPosition.greife(),
    "lass los": lambda: RoboterPosition.lass_los(),
    "sauger an": lambda: RoboterPosition.sauger_an(),
    "sauger aus": lambda: RoboterPosition.sauger_aus()
}

# --- Hauptfunktion zur Befehlsverarbeitung ---

def verarbeite_befehl(text):
    text = text.lower().strip()

    match = re.match(r"gehe zu (-?\d+\.?\d*) (-?\d+\.?\d*) (-?\d+\.?\d*)", text)
    if match:
        x, y, z = map(float, match.groups())
        RoboterPosition.schrittweise_bewegen(x, y, z)
        return RoboterPosition.aktueller_zustand()

    match = re.match(r"^(.*?)(?:\s(\d+))?$", text)
    if match:
        befehl = match.group(1).strip()
        anzahl = int(match.group(2)) if match.group(2) else 1

        if befehl in BEWEGUNGSBEFEHLE:
            achse, richtung = BEWEGUNGSBEFEHLE[befehl]
            if achse == "x":
                RoboterPosition.bewege_x(richtung * anzahl)
            elif achse == "y":
                RoboterPosition.bewege_y(richtung * anzahl)
            elif achse == "z":
                RoboterPosition.bewege_z(richtung * anzahl)
            return RoboterPosition.aktueller_zustand()

        elif befehl in SONSTIGE_BEFEHLE:
            SONSTIGE_BEFEHLE[befehl]()
            return RoboterPosition.aktueller_zustand()

    print("Befehl nicht erkannt:", text)
    return "UNKNOWN_COMMAND"

# --- Testen im Terminal ---

if __name__ == "__main__":
    while True:
        eingabe = input("Befehl: ")
        result = verarbeite_befehl(eingabe)
        print(result)























