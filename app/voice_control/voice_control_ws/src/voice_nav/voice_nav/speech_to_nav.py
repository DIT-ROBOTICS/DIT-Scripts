import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
import pyaudio
from vosk import Model, KaldiRecognizer

class SpeechToNavNode(Node):
    def __init__(self):
        super().__init__('speech_to_nav')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # 載入 VOSK 模型
        self.model = Model("/root/vosk_model/vosk-model-small-en-us-0.15")
        self.recognizer = KaldiRecognizer(self.model, 16000)

        # 目標座標
        self.goal_locations = {
            "point one": (2.3607, 0.8593, 0.0, 0.0, 0.7254, 0.6882),
            "point two": (1.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            "point a": (0.5, 1.5, 0.0, 0.0, -0.7, 0.7),
            "point b": (0.5, 0.85, 0.0, 0.0, -0.7, 0.7)
        }

        self.start_listening()

    def start_listening(self):
        """持續監聽語音"""
        self.get_logger().info("開始語音識別...")

        audio = pyaudio.PyAudio()
        stream = audio.open(format=pyaudio.paInt16, channels=1, rate=16000,
                            input=True, frames_per_buffer=8192)
        stream.start_stream()

        while rclpy.ok():
            data = stream.read(4096, exception_on_overflow=False)
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                text = result.get("text", "").lower()
                self.process_speech_command(text)

    def process_speech_command(self, text):
        """處理語音指令"""
        if not text:
            return

        self.get_logger().info(f"識別結果: {text}")

        for location, (x, y, z, ox, oy, ow) in self.goal_locations.items():
            if location in text:
                self.send_nav_goal(float(x), float(y), float(z), float(ox), float(oy), float(ow))
                return

    def send_nav_goal(self, x, y, z, ox, oy, ow):
        """發送導航目標"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = float(z)
        goal.pose.orientation.x = float(ox)
        goal.pose.orientation.y = float(oy)
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = float(ow)

        self.publisher_.publish(goal)
        self.get_logger().info(f"導航至 {x}, {y}")

def main():
    rclpy.init()
    node = SpeechToNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
