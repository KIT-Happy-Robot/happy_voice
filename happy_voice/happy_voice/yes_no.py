import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from happy_voice_msgs.action import YesNo
from happy_voice_msgs.srv import SpeechToText, TextToSpeech

class YesNoActionServer(Node):

    def __init__(self):
        super().__init__('yes_no_action_server')
        self._action_server = ActionServer(
            self,
            YesNo,
            'yes_no',
            self.execute_callback)
        self.tts_client = self.create_client(TextToSpeech, 'text_to_speech')
        self.stt_client = self.create_client(SpeechToText, 'speech_to_text')

        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("TTSサービス待機中...")
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("STTサービス待機中...")

    async def execute_callback(self, goal_handle):
        self.get_logger().info('📡 Executing goal...')
        feedback_msg = YesNo.Feedback()
        result = YesNo.Result()
        yes_words = {"yes", "yeah", "yep", "はい"}
        no_words = {"no", "nope", "nah", "いいえ"}

        MAX_RETRIES = 2
        retry_count = 0

        while retry_count <= MAX_RETRIES:
            # フィードバック送信 & TTS
            feedback_msg.current_phase = f"TTS phase (try {retry_count + 1})"
            goal_handle.publish_feedback(feedback_msg)

            tts_req = TextToSpeech.Request()
            tts_req.text = goal_handle.request.prompt_text if retry_count == 0 else "Sorry, one more time please."
            tts_req.wait_until_done = True
            tts_future = self.tts_client.call_async(tts_req)
            await tts_future

            # STT呼び出し
            feedback_msg.current_phase = f"STT phase (try {retry_count + 1})"
            goal_handle.publish_feedback(feedback_msg)

            stt_req = SpeechToText.Request()
            stt_future = self.stt_client.call_async(stt_req)
            await stt_future

            if not stt_future.result() or not stt_future.result().text:
                self.get_logger().warn("⚠️ STTが無効な結果を返しました")
                retry_count += 1
                continue

            text = stt_future.result().text.strip().lower().strip(".,!?")
            self.get_logger().info(f"📝 認識結果: '{text}'")

            if text in yes_words:
                result.result = True
                goal_handle.succeed()
                return result
            elif text in no_words:
                result.result = False
                goal_handle.succeed()
                return result
            else:
                self.get_logger().warn(f"⚠️ 不明な入力「{text}」、再試行")
                retry_count += 1

        # 最後まで判定できなかった
        self.get_logger().error("❌ 最大リトライに達したため中断")
        result.result = False
        goal_handle.abort()
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = YesNoActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()