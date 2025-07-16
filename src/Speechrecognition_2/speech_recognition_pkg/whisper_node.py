# speech_recognition_pkg/whisper_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from faster_whisper import WhisperModel
import pyaudio
import wave
import tempfile
import os

class WhisperNode(Node):

    def __init__(self):
        super().__init__('whisper_node')
        self.publisher_ = self.create_publisher(String, 'whisper_text', 10)

        self.get_logger().info('Whisper Node gestartet. Aufnahme beginnt...')

        # Whisper Modell laden
        self.model = WhisperModel("base", device="cpu", compute_type="int8")
        
        # Audioaufnahme starten
        self.record_and_transcribe()

    def record_and_transcribe(self):
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        CHUNK = 1024
        RECORD_SECONDS = 5

        p = pyaudio.PyAudio()

        # Einfach erstes Eingabegerät nehmen (oder spezifisch wählen)
        device_index = None
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                device_index = i
                self.get_logger().info(f"Audio Gerät gefunden: {info['name']} (Index {i})")
                break

        if device_index is None:
            self.get_logger().error("Kein Eingabegerät gefunden.")
            return

        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        input_device_index=device_index,
                        frames_per_buffer=CHUNK)

        frames = []
        for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK)
            frames.append(data)

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Speichern als temporäre WAV-Datei
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            wav_path = tmp.name
            wf = wave.open(wav_path, 'wb')
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(p.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
            wf.close()

        # Transkription mit Whisper
        segments, info = self.model.transcribe(wav_path, language="de")
        
        # Ergebnisse veröffentlichen
        for segment in segments:
            msg = String()
            msg.data = segment.text
            self.publisher_.publish(msg)
            self.get_logger().info(f"Gesendet: {segment.text}")

        os.remove(wav_path)

def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
