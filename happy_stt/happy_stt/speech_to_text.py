import rclpy
from rclpy.node import Node
from happy_voice_msgs.srv import SpeechToText
import requests

class STTServerNode(Node):
    def __init__(self):
        super().__init__('stt_server_node')
        self.srv = self.create_service(SpeechToText, 'speech_to_text', self.handle_stt)
        self.get_logger().info("🎧 Whisperノード起動！")

    def handle_stt(self, request, response):
        self.get_logger().info("🎧 Whisperに音声認識リクエスト送信中...")
        try:
            r = requests.post("http://localhost:5050/transcribe", timeout=30)
            response.text = r.json().get("text", "")
            self.get_logger().info(f"📝 認識結果: {response.text}")
        except Exception as e:
            self.get_logger().error(f"❌ エラー: {e}")
            response.text = ""
        return response

def main(args=None):
    rclpy.init(args=args)
    node = STTServerNode()
    rclpy.spin(node)
    rclpy.shutdown()
