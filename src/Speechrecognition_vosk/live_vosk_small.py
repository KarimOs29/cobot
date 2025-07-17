import sounddevice as sd
import queue
import json
from vosk import Model, KaldiRecognizer

# === VOSK-Spracherkennungsmodell laden ===
# Das Modell muss vorher lokal heruntergeladen und entpackt worden sein
model = Model("vosk-model-small-de-0.15")          # Deutsches Offline-Modell
recognizer = KaldiRecognizer(model, 16000)         # Initialisiert mit Modell und Samplerate

samplerate = 16000                                 # Abtastrate in Hz
q = queue.Queue()                                  # Queue zur Übergabe der Audio-Frames

# === Callback-Funktion: Audio wird live in die Queue geschrieben ===
def callback(indata, frames, time, status):
    if status:
        print("⚠️ Status:", status)                # Zeigt ggf. Warnungen von Sounddevice
    q.put(bytes(indata))                           # Audioframe als Bytes in die Queue legen

# === Sprachsteuerung: wartet auf "start", verarbeitet Sprache, stoppt bei "stopp" ===
def sprachsteuerung():
    print("⏳ Warte auf Sprachbefehl 'start'...")

    # Mikrofon-Eingabestream starten
    with sd.RawInputStream(samplerate=samplerate, blocksize=8000,
                           dtype='int16', channels=1, callback=callback):
        try:
            aktiviert = False                      # Anfangszustand: Spracheingabe deaktiviert

            while True:
                data = q.get()                     # Hole nächsten Audioblock aus der Queue

                if recognizer.AcceptWaveform(data):
                    result = recognizer.Result()   # JSON-Ergebnis von VOSK
                    text = json.loads(result)["text"].lower()  # Transkribierter Text (klein)

                    if not aktiviert:
                        if "start" in text:
                            print("🎙 Sprachsteuerung aktiviert – du kannst jetzt sprechen.")
                            aktiviert = True
                    else:
                        if text:
                            print("→", text)       # Ausgabe des gesprochenen Textes

                            if "stopp" in text:
                                print("🛑 Sprachbefehl 'stopp' erkannt. Aufnahme wird beendet.")
                                break

        except KeyboardInterrupt:
            print("\n🔌 Manuell beendet mit Strg+C")

    print("✅ Aufnahme sauber beendet.")

# === Hauptprogramm starten ===
sprachsteuerung()