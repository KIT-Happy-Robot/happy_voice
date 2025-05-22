import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from happy_voice_msgs.srv import SpeechToText, YesNo, TextToSpeech

class YesNoNode(Node):
    def __init__(self):
        super().__init__('yes_no_node')
        self.cb_group = ReentrantCallbackGroup()

        self.stt_client = self.create_client(SpeechToText, 'speech_to_text', callback_group=self.cb_group)
        self.tts_client = self.create_client(TextToSpeech, 'text_to_speech', callback_group=self.cb_group)
        self.srv = self.create_service(YesNo, 'yes_no', self.handle_yes_no, callback_group=self.cb_group)

        self.get_logger().info("🟡 サービス待機中: speech_to_text / text_to_speech")
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('🔄 speech_to_text サービスを待機中...')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('🔄 text_to_speech サービスを待機中...')
        self.get_logger().info("✅ すべてのサービスが利用可能です")

    def handle_yes_no(self, request, response):
        yes_words = {"yes", "yeah", "yep", "はい"}
        no_words = {"no", "nope", "nah", "いいえ"}

        while rclpy.ok():
            # TTSで案内
            if not self.speak("Please say yes or no."):
                response.result = False
                return response

            # STTサービス呼び出し
            self.get_logger().info("📨 speech_to_text サービス呼び出し中...")
            stt_req = SpeechToText.Request()
            future = self.stt_client.call_async(stt_req)

            # 非同期呼び出しの完了を待つ
            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    break

            if future.result() is None:
                self.get_logger().error("❌ STTの応答がありませんでした")
                response.result = False
                return response

            text = future.result().text.strip().lower().strip(".,!?")
            self.get_logger().info(f"📝 認識結果: '{text}'")

            if not text:
                self.get_logger().warn("⚠️ 空の認識結果。リトライします")
                self.speak("I could not hear anything. Please try again.")
                continue

            if text in yes_words:
                self.get_logger().info("🟢 YESと判定")
                response.result = True
                return response
            elif text in no_words:
                self.get_logger().info("🔴 NOと判定")
                response.result = False
                return response
            else:
                self.get_logger().warn(f"⚠️ 不明な応答「{text}」、再試行します")
                self.speak("Sorry, one more time please.")

    def speak(self, text):
        """TTSで発話し、成功すればTrueを返す"""
        tts_req = TextToSpeech.Request()
        tts_req.text = text
        tts_req.wait_until_done = True  # 🔑 発話終了まで待つ
        self.get_logger().info(f"🗣️ TTS発話: {text} (wait_until_done=True)")

        future = self.tts_client.call_async(tts_req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                break

        if future.result() and future.result().success:
            return True
        self.get_logger().error("❌ TTSが失敗しました")
        return False

def main():
    rclpy.init()
    node = YesNoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
