#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import os

class WhisperLauncher(Node):
    def __init__(self):
        super().__init__('whisper_launcher')
        self.get_logger().info("🚀 Whisper Dockerコンテナを起動中...")

        container_name = "whisper_server"
        image_name = "whisper:saved"

        # bind mount する app.py のホスト上パス（必要に応じて変更）
        host_app_path = "/home/mimi/docker_stt/app.py"
        container_app_path = "/opt/whisper/server/app.py"

        # パス存在チェック（念のため）
        if not os.path.exists(host_app_path):
            self.get_logger().error(f"❌ 指定された app.py が存在しません: {host_app_path}")
            return

        try:
            # Docker コンテナ起動 + bind mount 付きで app.py を差し替え
            self.process = subprocess.Popen([
                "docker", "run", "--rm",
                "--runtime", "nvidia",
                "--network=host",
                "--device", "/dev/snd",
                "--name", container_name,
                "-v", f"{host_app_path}:{container_app_path}",  # ← ここがbind mount！
                image_name,
                "bash", "-c", f"python3 {container_app_path}"
            ])
            self.get_logger().info("✅ Whisper Flaskサーバーを起動しました！（host networking）")

        except Exception as e:
            self.get_logger().error(f"❌ Whisperサーバーの起動に失敗: {e}")

    def destroy_node(self):
        self.get_logger().info("🛑 Whisperコンテナを終了中...")
        try:
            subprocess.run(["docker", "stop", "whisper_server"], check=False)
        except Exception as e:
            self.get_logger().warn(f"⚠️ コンテナ停止時にエラー: {e}")
        super().destroy_node()

def main():
    rclpy.init()
    node = WhisperLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
