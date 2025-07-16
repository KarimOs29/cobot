import pyaudio
import numpy as np
import soundfile as sf
import librosa
import io
import time
from faster_whisper import WhisperModel
from collections import deque

# === Whisper-Modell laden ===
model = WhisperModel("base", device="cpu", compute_type="int8")

# === Audioaufnahme-Einstellungen ===
FORMAT = pyaudio.paInt16
CHANNELS = 1
CHUNK = 1024
BLOCK_SECONDS = 4

p = pyaudio.PyAudio()

# === Mikrofon automatisch erkennen ===
DEVICE_INDEX = None
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    if info["maxInputChannels"] > 0:
        DEVICE_INDEX = i
        print(f"🎙 Verwende Mikrofon: {info['name']} (Index {i})")
        break

if DEVICE_INDEX is None:
    raise RuntimeError("Kein geeignetes Mikrofon gefunden.")

# === Abtastrate ermitteln und setzen ===
info = p.get_device_info_by_index(DEVICE_INDEX)
INPUT_RATE = int(info["defaultSampleRate"])  # z. B. 44100
TARGET_RATE = 16000  # für Whisper

print(f"🎚 Mikrofon-Sample-Rate: {INPUT_RATE} Hz")
print(f"🎯 Ziel-Sample-Rate für Whisper: {TARGET_RATE} Hz")

# === Stream öffnen ===
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=INPUT_RATE,
                input=True,
                input_device_index=DEVICE_INDEX,
                frames_per_buffer=CHUNK)

print("System bereit. Sage 'start' zum Aktivieren oder 'stopp' zum Beenden.")

buffer = deque()
block_size = int(INPUT_RATE * BLOCK_SECONDS / CHUNK)
aktiviert = False

try:
    while True:
        data = stream.read(CHUNK, exception_on_overflow=False)
        buffer.append(data)

        if len(buffer) >= block_size:
            frames = list(buffer)
            buffer.clear()

            audio_bytes = b''.join(frames)
            audio_np = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32) / 32768.0

            # === Resample auf 16000 Hz ===
            audio_np_resampled = librosa.resample(audio_np, orig_sr=INPUT_RATE, target_sr=TARGET_RATE)

            # === In WAV schreiben und transkribieren ===
            with io.BytesIO() as wav_io:
                sf.write(wav_io, audio_np_resampled, TARGET_RATE, format='WAV')
                wav_io.seek(0)

                segments, _ = model.transcribe(wav_io, language="de")
                full_text = ""
                for segment in segments:
                    full_text += segment.text.lower() + " "

                if "start" in full_text and not aktiviert:
                    aktiviert = True
                    print("Sprachsteuerung aktiviert.")
                    continue

                if "stopp" in full_text and aktiviert:
                    print("Sprachbefehl 'stopp' erkannt. Beende...")
                    break

                if aktiviert and full_text.strip():
                    print("→", full_text.strip())
                    time.sleep(1.5)

except KeyboardInterrupt:
    print("\n🔌 Manuell beendet mit Strg+C")

# Aufräumen
stream.stop_stream()
stream.close()
p.terminate()
print("Aufnahme sauber beendet.")
