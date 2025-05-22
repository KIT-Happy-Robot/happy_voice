import rclpy
from rclpy.node import Node
import subprocess
import os

class WhisperServerLauncher(Node):

    def __init__(self):
        super().__init__('whisper_server_launcher')
        self.get_logger().info("🚀 WhisperサーバーをDockerで起動します...")

        dockerfile_path = '/home/mimi/happy_ws/src/happy_voice/happy_stt'

        # docker build
        build_cmd = [
            "docker", "build", "--platform=linux/amd64", "-t", "whisper_server", dockerfile_path
        ]

        try:
            self.get_logger().info("🛠️ Dockerイメージをビルド中...")
            subprocess.run(build_cmd, check=True)
            self.get_logger().info("✅ Dockerビルド完了")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Docker build failed: {e}")
            return

        # docker run
        run_cmd = [
            "pasuspender", "--",
            "docker", "run", "--rm", "-i",
            "--device", "/dev/snd",
            "--network=host",
            "-v", os.path.expanduser("~/.asoundrc") + ":/root/.asoundrc",
            "--ipc=host",
            "whisper_server"
        ]

        try:
            self.get_logger().info("▶️ Dockerコンテナを起動中...")
            subprocess.run(run_cmd)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Docker run failed: {e}")
        except FileNotFoundError:
            self.get_logger().error("❌ pasuspender または docker が見つかりません。インストールされていますか？")


def main(args=None):
    rclpy.init(args=args)
    node = WhisperServerLauncher()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
