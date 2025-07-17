import pyaudio
import wave
import time
import os

# === Aufnahmeparameter ===
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
DAUER = 6 # Sekunden

# === Speicherpfad ins lokale Projektverzeichnis ===
output_path = os.path.join(os.path.dirname(__file__), "aufnahmen")
os.makedirs(output_path, exist_ok=True)

# === Dateinamen für die fünf Aufnahmen ===
dateien = [
    "audio_01.wav",
    "audio_02.wav",
    "audio_03.wav",
    "audio_04.wav",
    "audio_05.wav"
]

p = pyaudio.PyAudio()

print("🎙️ Aufnahme mit PyAudio startet...")

for dateiname in dateien:
    print(f"\nAufnahme {dateiname} – Sprich jetzt...")

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    frames = []
    for _ in range(0, int(RATE / CHUNK * DAUER)):
        data = stream.read(CHUNK)
        frames.append(data)

    stream.stop_stream()
    stream.close()

    dateipfad = os.path.join(output_path, dateiname)
    wf = wave.open(dateipfad, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    print(f"Gespeichert unter: {dateipfad}")
    time.sleep(1)

p.terminate()
print("\nAlle 5 Aufnahmen abgeschlossen.")
