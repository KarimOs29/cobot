#!/usr/bin/env python3

import sounddevice as sd
import queue
import json
import numpy as np
import threading
from vosk import Model, KaldiRecognizer
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# === Konfiguration ===
vosk_samplerate = 16000
q = queue.Queue()
stop_flag = threading.Event()

# === Mikrofonwahl: PulseAudio bevorzugt, sonst USB oder Fallback ===
def finde_mikro_index():
    try:
        sd.check_input_settings(device='pulse')
        print("🔍 Versuche PulseAudio zu verwenden ...")
        return 'pulse'
    except Exception:
        print("PulseAudio nicht verfügbar – verwende alternatives Gerät")

    usb_index = None
    fallback_index = None
    for idx, device in enumerate(sd.query_devices()):
        print(device['name'])
        if device['max_input_channels'] >= 1:
            name = device['name'].lower()
            if "usb" in name:
                usb_index = idx
                break
            elif fallback_index is None:
                fallback_index = idx

    if usb_index is not None:
        print(f"USB-Mikrofon gefunden: Index {usb_index}")
        return usb_index
    elif fallback_index is not None:
        print(f"Kein USB-Mikro, nutze anderes Mikrofon: Index {fallback_index}")
        return fallback_index
    else:
        raise RuntimeError("Kein Mikrofon gefunden")

# === Callback: Float32 → Resample → Int16 → Queue
def create_callback(native_sr):
    factor = vosk_samplerate / native_sr

    def callback(indata, frames, time, status):
        if status:
            print("⚠️ Status:", status)
        data = indata[:, 0] * 32767
        resampled = np.interp(
            np.arange(0, len(data), 1 / factor),
            np.arange(len(data)),
            data
        ).astype(np.int16)
        q.put(resampled.tobytes())
    return callback

# === Hauptfunktion ===
class SprachPublisher(Node):
    def __init__(self):
        super().__init__('sprach_publisher')  # ROS2 Node-Name
        self.publisher_ = self.create_publisher(String, 'sprachbefehl', 10)

    def starte_sprachsteuerung(self, befehl_callback):
        model = Model("vosk-model-small-de-0.15")
        recognizer = KaldiRecognizer(model, vosk_samplerate)

        print("Eingabegerät auswählen ...")
        device_index = finde_mikro_index()
        device_info = sd.query_devices(device_index, kind="input")
        native_sr = int(device_info['default_samplerate'])
        print(f"🎧 Gerät: {device_info['name']} – Sample Rate: {native_sr}Hz")

        def audio_verarbeitung():
            print("🎤 Sprachsteuerung dauerhaft aktiv!")

            while not stop_flag.is_set():
                try:
                    data = q.get(timeout=1)
                except queue.Empty:
                    continue

                if recognizer.AcceptWaveform(data):
                    text = json.loads(recognizer.Result()).get("text", "").lower().strip()
                    print(f"Erkannt: {text}")

                    match = re.search(r"\broboter\b (.+)", text)
                    if match:
                        text = match.group(1).strip()  # Nur das hintere wird in text gespeichert
                        print(f"Befehl erkannt: {text}")

                        msg = String()
                        msg.data = text
                        self.publisher_.publish(msg)
                        print('Befehl gesendet an Topic')

                        try:
                            from roboter_position import RoboterPosition
                            RoboterPosition.init_moveit()
                            RoboterPosition.aktualisiere_aus_moveit()
                        except Exception as e:
                            print(f"Fehler beim Positionsupdate: {e}")

                        befehl_callback(text)
                    else:
                        print(f"Ignoriert: {text}")


        thread = threading.Thread(target=audio_verarbeitung, daemon=True)
        thread.start()

        try:
            with sd.InputStream(
                samplerate=native_sr,
                blocksize=1024,
                latency='high',
                device=device_index,
                dtype='float32',
                channels=1,
                callback=create_callback(native_sr)
            ):
                while not stop_flag.is_set():
                    sd.sleep(100)
        except KeyboardInterrupt:
            print("Strg+C gedrückt – Sprachsteuerung wird gestoppt")
            stop_flag.set()

        thread.join()

        rclpy.shutdown()

        print("🛑 Sprachverarbeitung vollständig beendet.")

# === Startpunkt ===
if __name__ == "__main__":
    from roboter_position import verarbeite_befehl

    rclpy.init()

    sprach_publisher = SprachPublisher()
    sprach_publisher.starte_sprachsteuerung(verarbeite_befehl)

    sprach_publisher.destroy_node()

    rclpy.shutdown()