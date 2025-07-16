import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer

# === VOSK-Spracherkennungsmodell laden ===
# Pfad zum entpackten Modellordner (anpassen falls nötig!)
model = Model("vosk-model-de-0.21")
  # oder direkt "vosk-model-de-0.21", wenn kein Doppelfolder



device_index = 4  # HDA Intel PCH: CX20632 Alt Analog
device_info = sd.query_devices(device_index, 'input')
samplerate = int(device_info['default_samplerate'])
print(f"🎙️ Verwende Eingabegerät: {device_info['name']}")
print(f"📏 Samplerate: {samplerate} Hz")

recognizer = KaldiRecognizer(model, samplerate)              # Initialisiert mit Modell und Samplerate


q = queue.Queue()


# === Callback-Funktion: Audio wird live in die Queue geschrieben ===
def callback(indata, frames, time, status):
    if status:
        print("Status:", status)  # Zeigt ggf. Warnungen von Sounddevice
    q.put(bytes(indata))          # Audiodaten als Bytes in die Queue legen

# === Spracherkennung: wartet auf "start", verarbeitet Sprache, stoppt bei "stopp" ===
def sprachsteuerung():
    print("Spracheingabe gestartet – bitte sprich...")

    with sd.RawInputStream(device=device_index,
                       samplerate=samplerate,
                       blocksize=8000,
                       dtype='int16',
                       channels=1,
                       callback=callback):

        aktiviert = False

        while True:
            data = q.get()  # Hole nächsten Audioblock aus der Queue

            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                text = result.get("text", "")
                print("Erkannt:", text)

                if "start" in text.lower():
                    print("Startbefehl erkannt!")
                    aktiviert = True

                elif "stopp" in text.lower() or "stop" in text.lower():
                    print("Stoppbefehl erkannt. Beende...")
                    break

                elif aktiviert:
                    # Hier kannst du eigene Steuerbefehle einbauen
                    print("▶Sprachsteuerung aktiv:", text)

if __name__ == "__main__":
    sprachsteuerung()
