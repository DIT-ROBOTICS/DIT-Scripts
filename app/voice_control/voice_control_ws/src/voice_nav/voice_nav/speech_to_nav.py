import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import speech_recognition as sr

class SpeechToNavNode(Node):
    def __init__(self):
        super().__init__('speech_to_nav')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # 預設導航目標座標
        self.goal_locations = {
            "test": (2.3607, 0.8593, 0.7254, 0.6882, 0, 1.0),
            "beta": (1.0, 1.0, 0.0, 0.0, 0, 1.0),
            "echo": (-1.0, 2.0, -0.7, 0.7, 0, 1.0)
        }


    def listen_and_navigate(self):
        with self.microphone as source:
            self.get_logger().info("請說出目標地點（例如 'Sean'）...")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        try:
            # 設定關鍵字加權
            keywords = ["test", "beta", "echo"]
            text = self.recognizer.recognize_google(audio, language="en-US", show_all=False)

            text = text.lower()
            self.get_logger().info(f"語音識別結果: {text}")

            # 使用最接近的關鍵詞
            best_match = None
            for keyword in keywords:
                if keyword in text:
                    best_match = keyword
                    break

            if best_match:
                x, y, ox, oy, oz, ow = self.goal_locations[best_match]
                self.send_nav_goal(float(x), float(y), float(0.0), float(ox), float(oy), float(oz), float(ow))
            else:
                self.get_logger().warn("無法識別目標，請重試。")

        except sr.UnknownValueError:
            self.get_logger().warn("聽不懂，請再說一次。")
        except sr.RequestError as e:
            self.get_logger().warn(f"語音服務錯誤: {e}")




    def send_nav_goal(self, x, y, z, ox, oy, oz, ow):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(z)  # 確保 z 是 float
        goal.pose.orientation.x = float(ox)
        goal.pose.orientation.y = float(oy)
        goal.pose.orientation.z = float(oz)
        goal.pose.orientation.w = float(ow)

        self.publisher_.publish(goal)
        self.get_logger().info(f"發送導航目標: x={x}, y={y}, z={z}")


def main():
    rclpy.init()
    node = SpeechToNavNode()

    while rclpy.ok():
        node.listen_and_navigate()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
