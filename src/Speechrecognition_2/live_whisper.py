import pyaudio   #für audioaufnahme mit mikrophon
import wave
import tempfile
import os
from faster_whisper import WhisperModel

# === Whisper-Modell laden ===
model_size = "base"     #legt fest welches whisper modell geladen wird(trade genauigkeit/geschwindigkeit)
model = WhisperModel(model_size, device="cpu", compute_type="int8")   

# === Aufnahme-Einstellungen ===
FORMAT = pyaudio.paInt16
CHANNELS = 1    #1 Mikrofonkanal
RATE = 16000    #Abtastrate
CHUNK = 1024    #Chunk ist ein Buffer,Pyaudio liest hier 1024 Datenblöcke auf einmal
RECORD_SECONDS = 5    

# === Aufnahmegerät automatisch erkennen ===
p = pyaudio.PyAudio()    #erstellen von Pyaudio-Objekt
DEVICE_INDEX = None


#Schleife über alle gefundenen Geräte sobald eins den Namen pulse hat und mehr als ein Eingangskanal wird dies genutzt 
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    name = info["name"].lower()
    if "pulse" in name and info["maxInputChannels"] > 0:
        DEVICE_INDEX = i
        print(f"Pulse-Gerät gefunden: {info['name']} (Index {i})")
        break

if DEVICE_INDEX is None:
    raise RuntimeError("Kein geeignetes PulseAudio-Eingabegerät gefunden.")

# === Audioaufnahme starten ===
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index=DEVICE_INDEX,
                frames_per_buffer=CHUNK)

print("Aufnahme gestartet. Sprich jetzt...")

frames = []
for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("Aufnahme beendet.")
stream.stop_stream()
stream.close()
p.terminate()

# === WAV-Datei temporär speichern ===
with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
    wav_path = tmp.name
    wf = wave.open(wav_path, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

# === Transkription mit Whisper ===
print("Transkription läuft...")

segments, info = model.transcribe(wav_path, language="de")

for segment in segments:
    print(f"[{segment.start:.2f}s – {segment.end:.2f}s]: {segment.text}")

# === Temporäre Datei löschen ===
os.remove(wav_path)

print("Fertig.")
