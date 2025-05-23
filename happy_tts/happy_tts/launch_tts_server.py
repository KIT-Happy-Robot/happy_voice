#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import os

class PiperLauncher(Node):
    def __init__(self):
        super().__init__('piper_launcher')
        self.get_logger().info("🚀 Launching Piper server on port 5001...")

        try:
            os.chdir('/home/mimi/piper')  # 必要なら変更

            self.process = subprocess.Popen([
                "docker", "run", "--rm",
                "--network=host",
                "-v", f"{os.getcwd()}/models:/app/models",
                "piper-server",
                "bash", "-c",
                "python3 -m piper.http_server "
                "--model /app/models/en_US-hfc_female-medium.onnx "
                "--config /app/models/en_US-hfc_female-medium.onnx.json "
                "--port 5000"
            ])
            self.get_logger().info("✅ Piper server started on port 5001 (host networking).")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start Piper: {e}")

def main():
    rclpy.init()
    node = PiperLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
