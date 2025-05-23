import rclpy
from rclpy.node import Node
import subprocess
import os

class WhisperServerLauncher(Node):

    def __init__(self):
        super().__init__('whisper_server_launcher')
        self.get_logger().info("🚀 WhisperサーバーをDockerで起動します...")

        dockerfile_path = '/home/mimi/happy_ws/src/happy_voice/happy_stt'
        container_name = "whisper_container"
        image_name = "whisper_server"
        host_app_path = dockerfile_path
        container_app_path = "/app/app.py"

        # # docker build
        # build_cmd = [
        #     "docker", "build", "-t", "whisper_server", dockerfile_path
        # ]

        try:
            self.get_logger().info("🛠️ Dockerイメージをビルド中...")
            # subprocess.run(build_cmd, check=True)
            self.get_logger().info("✅ Dockerビルド完了")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Docker build failed: {e}")
            return

        # docker run
        run_cmd = [
        "pasuspender", "--",                      # PulseAudioの音声排他
        "docker", "run", "--rm", "-i",
        "--runtime", "nvidia",                    # ← CUDAを有効化
        "--device", "/dev/snd",                   # 音声デバイス
        "--network=host",                         # ホストネットワーク
        "--ipc=host",                             # IPC共有（音声録音時に必要な場合あり）
        "--name", "whisper_container",            # 任意のコンテナ名
        "-v", os.path.expanduser("~/.asoundrc") + ":/root/.asoundrc",  # 音声設定の共有
        "-v", f"{host_app_path}:/app",            # アプリコードのマウント
        "whisper_server",                         # イメージ名
        "bash", "-c", "python3 /app/app.py"       # 起動コマンド
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