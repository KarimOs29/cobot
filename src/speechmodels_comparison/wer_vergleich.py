import os
import pandas as pd
from jiwer import wer

# === Verzeichnisse ===
whisper_dir = "./whisper_ergebnisse"
vosk_dir = "./vosk_ergebnisse"

# === originaltexte ===
originaltexte = {
    "audio_01": "hey roboter, starten",
    "audio_02": "bewege dich nach rechts und danach nach links",
    "audio_03": "hey roboter, ich möchte dass du jetzt die tasse aufhebst",
    "audio_04": "hey roboter, kannst du folgende befehle ausführen",
    "audio_05": "okey roboter, jetzt sind wir fertig bitte beende"
}

# === Vergleichsdaten sammeln ===
vergleichsdaten = []

for name, ref in originaltexte.items():
    whisper_path = os.path.join(whisper_dir, f"{name}.txt")
    vosk_path = os.path.join(vosk_dir, f"{name}.txt")

    if not (os.path.exists(whisper_path) and os.path.exists(vosk_path)):
        continue

    with open(whisper_path, "r", encoding="utf-8") as f:
        whisper = f.read().strip().lower()

    with open(vosk_path, "r", encoding="utf-8") as f:
        vosk = f.read().strip().lower()

    wer_whisper = round(wer(ref, whisper), 3)
    wer_vosk = round(wer(ref, vosk), 3)

    vergleichsdaten.append({
        "Datei": name,
        "Referenztext": ref,
        "Whisper-Ausgabe": whisper,
        "WER Whisper": wer_whisper,
        "VOSK-Ausgabe": vosk,
        "WER VOSK": wer_vosk
    })

# === Als Tabelle speichern ===
df = pd.DataFrame(vergleichsdaten)
df.to_csv("wer_vergleich_tabelle.csv", index=False, encoding="utf-8")
print("✅ Vergleichstabelle mit WER gespeichert als wer_vergleich_tabelle.csv")
