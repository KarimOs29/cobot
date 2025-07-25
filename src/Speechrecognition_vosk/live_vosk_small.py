import sounddevice as sd
import queue
import json
import numpy as np
import threading
from vosk import Model, KaldiRecognizer

# === Konfiguration ===
vosk_samplerate = 16000
q = queue.Queue()
stop_flag = threading.Event()

# === Mikrofonwahl: PulseAudio bevorzugt, sonst USB oder Fallback ===
def finde_mikro_index():
    try:
        # PulseAudio wird unterstützt, aber nicht als Gerät gelistet – deshalb zuerst testen
        sd.check_input_settings(device='pulse')
        print("🔍 Versuche PulseAudio zu verwenden ...")
        return 'pulse'
    except Exception:
        print("⚠️ PulseAudio nicht verfügbar – verwende alternatives Gerät")

    usb_index = None
    fallback_index = None
    for idx, device in enumerate(sd.query_devices()):
        if device['max_input_channels'] >= 1:
            name = device['name'].lower()
            if "usb" in name:
                usb_index = idx
                break
            elif fallback_index is None:
                fallback_index = idx

    if usb_index is not None:
        print(f"🔌 USB-Mikrofon gefunden: Index {usb_index}")
        return usb_index
    elif fallback_index is not None:
        print(f"🎤 Kein USB-Mikro, nutze anderes Mikrofon: Index {fallback_index}")
        return fallback_index
    else:
        raise RuntimeError("❌ Kein Mikrofon gefunden")

# === Callback: Float32 → Resample → Int16 → Queue
def create_callback(native_sr):
    factor = vosk_samplerate / native_sr

    def callback(indata, frames, time, status):
        if status:
            print("⚠️ Status:", status)
        # Mono, float32 → int16
        data = indata[:, 0] * 32767
        # Resample (linear)
        resampled = np.interp(
            np.arange(0, len(data), 1 / factor),
            np.arange(len(data)),
            data
        ).astype(np.int16)
        q.put(resampled.tobytes())
    return callback

# === Verarbeitungs-Thread ===
def audio_verarbeitung(recognizer):
    aktiviert = False
    print("⏳ Warte auf Sprachbefehl 'starten'...")
    while not stop_flag.is_set():
        try:
            data = q.get(timeout=1)
        except queue.Empty:
            continue

        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            text = result.get("text", "").lower()

            if not aktiviert and "starten" in text:
                print("🎙 Sprachsteuerung aktiviert – du kannst jetzt sprechen.")
                aktiviert = True
            elif aktiviert:
                if text:
                    print("→", text)
                    if "stoppen" in text:
                        print("🛑 Sprachbefehl 'stoppen' erkannt.")
                        stop_flag.set()
                        break

# === Hauptfunktion ===
def sprachsteuerung():
    print("📦 Lade VOSK-Modell ...")
    model = Model("vosk-model-small-de-0.15")
    recognizer = KaldiRecognizer(model, vosk_samplerate)

    print("🔍 Eingabegerät auswählen ...")
    device_index = finde_mikro_index()
    device_info = sd.query_devices(device_index, kind="input")
    native_sr = int(device_info['default_samplerate'])
    print(f"🎙 Gerät: {device_info['name']} – Sample Rate: {native_sr} Hz")

    # Verarbeitungs-Thread starten
    thread = threading.Thread(target=audio_verarbeitung, args=(recognizer,), daemon=True)
    thread.start()

    try:
        with sd.InputStream(samplerate=native_sr,
                            blocksize=1024,        # kleinerer Block hilft gegen Overflow
                            latency='high',        # oder z.B. latency=0.2
                            device=device_index,
                            dtype='float32',
                            channels=1,
                            callback=create_callback(native_sr)):
            while not stop_flag.is_set():
                sd.sleep(100)
    except KeyboardInterrupt:
        print("🔌 Strg+C gedrückt – stoppen...")
        stop_flag.set()
    thread.join()
    print("✅ Aufnahme beendet.")

# === Startpunkt ===
if __name__ == "__main__":
    sprachsteuerung()
