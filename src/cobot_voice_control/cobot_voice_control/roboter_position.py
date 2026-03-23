#!/usr/bin/env python3
import re
import sys

class RoboterPosition:

    # Aktuelle Position
    x = 0.0
    y = 0.0
    z = 0.0

    # Aktorzustände
    greifer_status = "offen"
    sauger_status = "aus"

    @classmethod
    def bewege_x(cls, delta):
        cls.x = delta
        cls.y = 0
        cls.z = 0
        
    @classmethod
    def bewege_y(cls, delta):
        cls.x = 0
        cls.y = delta
        cls.z = 0

    @classmethod
    def bewege_z(cls, delta):
        cls.x = 0
        cls.y = 0
        cls.z = delta

    @classmethod
    def aktuelle_position(cls):
        return (cls.x, cls.y, cls.z)

    @classmethod
    def aktueller_zustand(cls):
        return {
            "befehl": "bewegung",
            "position": cls.aktuelle_position(),
            "greifer": cls.greifer_status,
            "sauger": cls.sauger_status
        }

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
                    return {"befehl": "UNKNOWN_COMMAND"}

                return RoboterPosition.aktueller_zustand()

        # Sonstige Befehle
        for key, argument in SONSTIGE_BEFEHLE.items():
            if befehl.startswith(key):
                print('Befehl erkannt: ', key)
                return {"befehl": argument}
            
        print("Befehl nicht erkannt:", text)
        return {"befehl": "UNKNOWN_COMMAND"}
    
    
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

