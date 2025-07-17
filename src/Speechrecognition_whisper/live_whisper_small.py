import pyaudio
import numpy as np
import soundfile as sf
import librosa
import io
import time
from faster_whisper import WhisperModel
from collections import deque

# === Whisper-Modell laden (genauer als 'base') ===
model = WhisperModel("small", device="cpu", compute_type="int8")

# === Audioaufnahme-Einstellungen ===
FORMAT = pyaudio.paInt16
CHANNELS = 1
CHUNK = 2048
BLOCK_SECONDS = 3  # mehr Kontext → bessere Erkennung

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
TARGET_RATE = 16000  # Whisper erwartet 16 kHz

print(f"🎚 Mikrofon-Sample-Rate: {INPUT_RATE} Hz")
print(f"🎯 Ziel-Sample-Rate für Whisper: {TARGET_RATE} Hz")

# === Stream öffnen ===
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=INPUT_RATE,
                input=True,
                input_device_index=DEVICE_INDEX,
                frames_per_buffer=CHUNK)

print("🟢 System bereit. Sage 'start' zum Aktivieren oder 'stopp' zum Beenden.")

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

            # === Resample auf 16 kHz ===
            audio_np_resampled = librosa.resample(audio_np, orig_sr=INPUT_RATE, target_sr=TARGET_RATE)

            # === WAV schreiben → Whisper analysieren ===
            with io.BytesIO() as wav_io:
                sf.write(wav_io, audio_np_resampled, TARGET_RATE, format='WAV')
                wav_io.seek(0)

                segments, _ = model.transcribe(wav_io, language="de")
                full_text = " ".join(segment.text.lower().strip() for segment in segments)

                if not full_text.strip():
                    continue  # keine Sprache erkannt

                if not aktiviert:
                    if "start" in full_text:
                        aktiviert = True
                        print("🟢 Sprachsteuerung aktiviert")
                    continue  # bis "start" ignorieren

                if "stopp" in full_text or "stop" in full_text:
                    print("🛑 Sprachsteuerung beendet")
                    break

                print(full_text.strip())
                

except KeyboardInterrupt:
    print("\n🔌 Manuell beendet mit Strg+C")

# === Aufräumen ===
stream.stop_stream()
stream.close()
p.terminate()
print("✅ Aufnahme sauber beendet.")
