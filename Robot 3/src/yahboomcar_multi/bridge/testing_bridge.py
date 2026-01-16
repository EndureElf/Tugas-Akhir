import rclpy
from rclpy.node import Node
import socket
import json
import threading


class Robot2UDPTest(Node):
    def __init__(self):
        super().__init__('robot2_udp_test')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8891))

        self.thread = threading.Thread(target=self.listen)
        self.thread.daemon = True
        self.thread.start()

        self.get_logger().info("🤖 Robot2 UDP TEST listening")

    def listen(self):
        while rclpy.ok():
            data, addr = self.sock.recvfrom(1024)
            msg = json.loads(data.decode())

            self.get_logger().info(
                f"📥 From Laptop ({addr[0]}): {msg}"
            )


def main():
    rclpy.init()
    node = Robot2UDPTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
