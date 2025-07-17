import os
import io
import time
import psutil
import numpy as np
import soundfile as sf
from scipy.signal import resample
from faster_whisper import WhisperModel

# === Ressourcenmessung vorbereiten ===
prozess = psutil.Process(os.getpid())
startzeit = time.time()

# === Pfade ===
eingabe_ordner = "./aufnahmen"
ausgabe_ordner = "./whisper_ergebnisse"
os.makedirs(ausgabe_ordner, exist_ok=True)

# === Whisper-Modell laden ===
model = WhisperModel("base", device="cpu", compute_type="int8")

# === Zielabtastrate ===
TARGET_RATE = 16000

# === WAV-Dateien durchgehen ===
for dateiname in sorted(os.listdir(eingabe_ordner)):
    if not dateiname.endswith(".wav"):
        continue

    eingabepfad = os.path.join(eingabe_ordner, dateiname)
    print(f" Verarbeite: {eingabepfad}")

    # === Audio laden ===
    audio_np, sr = sf.read(eingabepfad)
    if audio_np.ndim > 1:
        audio_np = np.mean(audio_np, axis=1)  # Stereo → Mono

    # === Resampling (falls nötig) ===
    if sr != TARGET_RATE:
        num_samples = int(len(audio_np) * TARGET_RATE / sr)
        audio_np = resample(audio_np, num_samples)

    # === In Memory WAV schreiben ===
    with io.BytesIO() as wav_io:
        sf.write(wav_io, audio_np, TARGET_RATE, format='WAV')
        wav_io.seek(0)

        # === Transkription
        segments, _ = model.transcribe(wav_io, language="de")
        full_text = " ".join([s.text.strip().lower() for s in segments])

        # === Ergebnis speichern
        name_ohne_endung = os.path.splitext(dateiname)[0]
        ausgabepfad = os.path.join(ausgabe_ordner, f"{name_ohne_endung}.txt")
        with open(ausgabepfad, "w", encoding="utf-8") as f:
            f.write(full_text)

        print(f" Transkript gespeichert: {ausgabepfad}")

# === Ressourcenmessung abschließen ===
endzeit = time.time()
dauer = endzeit - startzeit
ram = prozess.memory_info().rss / (1024 ** 2)

# === Ressourcenwerte speichern ===
with open("ressourcen_whisper.txt", "w") as f:
    f.write(f"Laufzeit: {dauer:.2f} Sekunden\n")
    f.write(f"RAM-Verbrauch (am Ende): {ram:.2f} MB\n")

print("\n Ressourcenverbrauch (WHISPER)):")
print(f" Gesamtzeit: {dauer:.2f} Sekunden")
print(f" RAM-Verbrauch (am Ende): {ram:.2f} MB")
