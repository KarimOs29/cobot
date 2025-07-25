import os
import io
import json
import time
import psutil
import numpy as np
import soundfile as sf
from scipy.signal import resample
from vosk import Model, KaldiRecognizer

# === Ressourcenmessung vorbereiten ===
prozess = psutil.Process(os.getpid())
startzeit = time.time()
speicher_vorher = prozess.memory_info().rss / (1024 * 1024)  # in MB

# === Pfade ===
eingabe_ordner = "./aufnahmen"
ausgabe_ordner = "./vosk_ergebnisse"
os.makedirs(ausgabe_ordner, exist_ok=True)

# === Vosk-Modell laden ===
model = Model("vosk-model-small-de-0.15")
samplerate = 16000

# === WAV-Dateien verarbeiten ===
for dateiname in sorted(os.listdir(eingabe_ordner)):
    if not dateiname.endswith(".wav"):
        continue

    eingabepfad = os.path.join(eingabe_ordner, dateiname)
    print(f" Verarbeite mit VOSK: {eingabepfad}")

    # Audio laden
    audio_np, sr = sf.read(eingabepfad)
    if audio_np.ndim > 1:
        audio_np = np.mean(audio_np, axis=1)  # Stereo → Mono

    # Resampling (falls nötig)
    if sr != samplerate:
        num_samples = int(len(audio_np) * samplerate / sr)
        audio_np = resample(audio_np, num_samples)

    # In Bytes für VOSK
    audio_bytes = (audio_np * 32768).astype(np.int16).tobytes()

    # Erkennung starten
    recognizer = KaldiRecognizer(model, samplerate)
    recognizer.AcceptWaveform(audio_bytes)
    result = json.loads(recognizer.Result())
    text = result.get("text", "").strip().lower()

    # Ergebnis speichern
    name_ohne_endung = os.path.splitext(dateiname)[0]
    ausgabepfad = os.path.join(ausgabe_ordner, f"{name_ohne_endung}.txt")
    with open(ausgabepfad, "w", encoding="utf-8") as f:
        f.write(text)

    print(f" Transkript gespeichert: {ausgabepfad}")

# === Ressourcenmessung abschließen ===
dauer = time.time() - startzeit
speicher_nachher = prozess.memory_info().rss / (1024 * 1024)
differenz = speicher_nachher - speicher_vorher

print("\n Ressourcenverbrauch (VOSK):")
print(f" Gesamtzeit: {dauer:.2f} Sekunden")
print(f" RAM-Verbrauch (am Ende): {differenz:.2f} MB")
