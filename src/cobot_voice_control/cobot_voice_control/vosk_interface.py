#!/usr/bin/env python3

import json
import queue
import re
import threading

import numpy as np
import rclpy
import sounddevice as sd
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

# === Konfiguration ===
VOSK_SAMPLERATE = 16000
MODEL_PATH = "vosk-model-small-de-0.15"


# === Mikrofonwahl: PulseAudio bevorzugt, sonst USB oder Fallback ===
def finde_mikro_index():
    try:
        sd.check_input_settings(device='pulse')
        print("Versuche PulseAudio zu verwenden ...")
        return 'pulse'
    except Exception:
        pass

    usb_index = None
    fallback_index = None

    for idx, device in enumerate(sd.query_devices()):
        if device.get("max_input_channels", 0) >= 1:
            name = str(device.get("name", "")).lower()
            print(f"[{idx}] {device.get('name', '')}")

            if "usb" in name:
                usb_index = idx
                break
            if fallback_index is None:
                fallback_index = idx

    if usb_index is not None:
        return usb_index
    if fallback_index is not None:
        return fallback_index

    raise RuntimeError("Kein Mikrofon gefunden (kein Input-Device verfügbar).")

# === Callback: Float32 → Resample → Int16 → Queue
def create_callback(native_sr: int, audio_queue: queue.Queue):
    factor = VOSK_SAMPLERATE / float(native_sr)

    def callback(indata, frames, time_info, status):
        if status:
            print("Audio-Status:", status)
            pass

        data = indata[:, 0] * 32767.0
        
        x_old = np.arange(len(data))
        x_new = np.arange(0, len(data), 1.0 / factor)
        resampled = np.interp(x_new, x_old, data).astype(np.int16)
        audio_queue.put(resampled.tobytes())

    return callback

# === Hauptfunktion ===
class SprachPublisher(Node):
    def __init__(self):
        super().__init__('sprach_publisher')  # ROS2 Node-Name
        self.publisher_ = self.create_publisher(String, 'sprachbefehl', 10)
        self._queue = queue.Queue()
        self._stop = threading.Event()
    
    def stop(self):
        self._stop.set()

    def run(self):
        model = Model(MODEL_PATH)
        recognizer = KaldiRecognizer(model, VOSK_SAMPLERATE)

        print("Eingabegerät auswählen ...")
        device_index = finde_mikro_index()
        device_info = sd.query_devices(device_index, kind="input")
        native_sr = int(device_info['default_samplerate'])
                
        self.get_logger().info(
            f"Mic: {device_info['name']} | native_sr={native_sr} | target_sr={VOSK_SAMPLERATE}"
        )

        def audio_loop():
            self.get_logger().info("Sprachsteuerung aktiv. Trigger: 'roboter ...'")
            while not self._stop.is_set() and rclpy.ok():
                try:
                    data = self._queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                if recognizer.AcceptWaveform(data):
                    text = json.loads(recognizer.Result()).get("text", "").lower().strip()
                    if not text:
                        continue
                
                    match = re.search(r"\broboter\b (.+)", text)
                    if not match:
                        continue
                    
                    command = match.group(1).strip() 
                    print(f"Befehl erkannt: {text}")

                    msg = String()
                    msg.data = command
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'→ Publish "sprachbefehl": {command}')
            
        worker = threading.Thread(target=audio_loop, daemon=True)
        worker.start()

        try:
            with sd.InputStream(
                samplerate=native_sr,
                blocksize=1024,
                latency='high',
                device=device_index,
                dtype='float32',
                channels=1,
                callback=create_callback(native_sr, self._queue),
            ):
                while not self._stop.is_set() and rclpy.ok():
                    sd.sleep(100)
        except KeyboardInterrupt:
            self.get_logger().info("Sprachsteuerung gestoppt")
        finally:
            self._stop.set()
            worker.join(timeout=2.0)
            self.get_logger().info("Sprachpublisher beendet.")
        
def main():
    rclpy.init()
    node = SprachPublisher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

# === Startpunkt ===
if __name__ == "__main__":
    main()