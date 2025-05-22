import rclpy
from rclpy.node import Node
from happy_voice_msgs.srv import SpeechToText

import requests

class SpeechToTextClient(Node):

    def __init__(self):
        super().__init__('speech_to_text_client')
        self.srv = self.create_service(SpeechToText, 'speech_to_text', self.speech_to_text_callback)
        self.get_logger().info("SpeechToTextClient")

    def speech_to_text_callback(self, request, response):
        self.get_logger().info('📡 Whisperサーバーにリクエスト送信中...')

        try:
            r = requests.post("http://localhost:5050/transcribe", timeout=30)
            r.raise_for_status()
            result = r.json()
            response.text = result.get("text", "")
            self.get_logger().info(f'📝 認識結果: {response.text}')
        except Exception as e:
            self.get_logger().error(f'❌ エラー: {e}')
            response.text = ""

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
