import pyaudio
import wave

# === Aufnahme-Einstellungen ===
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
RECORD_SECONDS = 5
OUTPUT_FILENAME = "testaufnahme.wav"
DEVICE_INDEX = 25  # oder 27 (pulse / default)

# === Aufnahme starten ===
p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK,
                input_device_index=DEVICE_INDEX)

print("Aufnahme läuft...")

frames = []
for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("Aufnahme beendet.")

stream.stop_stream()
stream.close()
p.terminate()

# === WAV-Datei speichern ===
wf = wave.open(OUTPUT_FILENAME, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(p.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
wf.close()

print(f"Gespeichert als {OUTPUT_FILENAME}")
