import sounddevice as sd
import queue
import json
import numpy as np
import time
from vosk import Model, KaldiRecognizer
from scipy.signal import resample

# === Modell laden ===
model = Model("vosk-model-de-0.21")

# === Mikrofon-Setup ===
device_index = 2  # ggf. anpassen
device_info = sd.query_devices(device_index, 'input')
mic_samplerate = int(device_info['default_samplerate'])
vosk_samplerate = 16000

print(f"Mikrofon: {device_info['name']}")
print(f"Mikrofon-Samplerate: {mic_samplerate} Hz → Ziel: {vosk_samplerate} Hz")

recognizer = KaldiRecognizer(model, vosk_samplerate)  #initalisierung der spracherkennung mit dem  model und der samplerate
q = queue.Queue()

def callback(indata, frames, time, status):
    if status:
        print( status)
    q.put(indata.copy())

def sprachsteuerung():
    print("Sprich 'starten' zum Starten der Erkennung")

    last_text = ""
    gestartet = False

#hier wird der eigentliche audiostream mit den gewählten parametern geöffnet
    with sd.InputStream(
        device=device_index,
        samplerate=mic_samplerate,
        blocksize=2048,
        dtype='float32',
        channels=1,
        latency='low',
        callback=callback
    ):

    #hier wird immer der nächste audioblock aus der queue geholt
        while True:
            data = q.get()
            data = np.squeeze(data)

            num_samples = int(len(data) * vosk_samplerate / mic_samplerate)
            resampled = resample(data, num_samples)
            resampled_bytes = (resampled * 32768).astype(np.int16).tobytes()
            

        #falls text erkannt wird
            if recognizer.AcceptWaveform(resampled_bytes):
                result = json.loads(recognizer.Result())
                text = result.get("text", "").lower().strip()

        #falls kein oder gleicher text erkannt wird
                if not text or text == last_text:
                    continue

                last_text = text

                if not gestartet:
                    if "starten" in text:
                        gestartet = True
                        print("Erkennung gestartet")
                    continue  # alles ignorieren, bis gestartet

                if "stoppen" in text or "stopp" in text:
                    print("Erkennung gestoppt")
                    break
        
        #hier findet die ausgabe statt und eine kleine pause eingebaut damit es sich nicht überhaspelt
                print(text)
                time.sleep(0.2)

if __name__ == "__main__":
    sprachsteuerung()
