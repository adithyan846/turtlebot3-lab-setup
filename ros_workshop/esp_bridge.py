
#!/usr/bin/env python3

import socket
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# =====================================================
# CONFIGURATION
# =====================================================

LOCAL_UDP_PORT = 8889

ESP32_UDP_PORT = 8888

# Maximum movement values represented by 100%.
MAX_LINEAR_MPS = 0.30
MAX_ANGULAR_RPS = 1.50

SEND_RATE_HZ = 20.0

# Stop sending an old command if the controller
# stops publishing new commands.
COMMAND_FRESHNESS_TIMEOUT = 0.5


class ESP32Bridge(Node):

    def __init__(self):

        super().__init__('esp32_udp_bridge')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        self.declare_parameter(
            'esp32_ip',
            '10.187.140.177'
        )

        self.esp32_ip = (
            self.get_parameter('esp32_ip').value
        )

        self.esp32_address = (
            self.esp32_ip,
            ESP32_UDP_PORT
        )

        # -------------------------------------------------
        # UDP SOCKET
        # -------------------------------------------------

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        # Bind to a fixed local port.
        # ESP32 learns this port from incoming commands.
        self.sock.bind(
            ('0.0.0.0', LOCAL_UDP_PORT)
        )

        # Non-blocking receive.
        self.sock.setblocking(False)

        # -------------------------------------------------
        # ROS 2 SUBSCRIBER
        # -------------------------------------------------

        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # -------------------------------------------------
        # ROS 2 PUBLISHER
        # -------------------------------------------------

        self.distance_publisher = self.create_publisher(
            Float32,
            '/ultrasonic/distance_cm',
            10
        )

        # -------------------------------------------------
        # COMMAND STORAGE
        # -------------------------------------------------

        self.latest_cmd = Twist()
        self.last_cmd_time = None

        # -------------------------------------------------
        # TIMER
        # -------------------------------------------------

        self.timer = self.create_timer(
            1.0 / SEND_RATE_HZ,
            self.timer_callback
        )

        self.get_logger().info(
            'ESP32 UDP bridge started.'
        )

        self.get_logger().info(
            f'ESP32 address: {self.esp32_ip}:{ESP32_UDP_PORT}'
        )

        self.get_logger().info(
            f'Listening on UDP port {LOCAL_UDP_PORT}'
        )

    # =================================================
    # RECEIVE ROS 2 MOVEMENT COMMAND
    # =================================================

    def cmd_callback(self, msg):

        self.latest_cmd = msg
        self.last_cmd_time = time.monotonic()

    # =================================================
    # CONVERT ROS VELOCITY TO PERCENTAGE
    # =================================================

    @staticmethod
    def clamp(value, minimum, maximum):

        return max(
            minimum,
            min(value, maximum)
        )

    def convert_velocity(self):

        # Stop if the controller has stopped publishing.
        if self.last_cmd_time is None:
            return 0.0, 0.0

        age = time.monotonic() - self.last_cmd_time

        if age > COMMAND_FRESHNESS_TIMEOUT:
            return 0.0, 0.0

        linear_mps = self.latest_cmd.linear.x
        angular_rps = self.latest_cmd.angular.z

        # Convert m/s into percentage.
        linear_percent = (
            linear_mps / MAX_LINEAR_MPS
        ) * 100.0

        # Convert rad/s into percentage.
        angular_percent = (
            angular_rps / MAX_ANGULAR_RPS
        ) * 100.0

        linear_percent = self.clamp(
            linear_percent,
            -100.0,
            100.0
        )

        angular_percent = self.clamp(
            angular_percent,
            -100.0,
            100.0
        )

        return linear_percent, angular_percent

    # =================================================
    # SEND COMMAND TO ESP32
    # =================================================

    def send_command(self):

        linear, angular = self.convert_velocity()

        message = (
            f'CMD,{linear:.1f},{angular:.1f}'
        )

        self.sock.sendto(
            message.encode('utf-8'),
            self.esp32_address
        )

    # =================================================
    # RECEIVE SENSOR DATA FROM ESP32
    # =================================================

    def receive_sensor_data(self):

        while True:

            try:

                data, address = self.sock.recvfrom(256)

            except BlockingIOError:

                break

            try:

                message = data.decode('utf-8').strip()

                fields = message.split(',')

                if len(fields) != 2:
                    continue

                if fields[0] != 'DIST':
                    continue

                distance = float(fields[1])

                if not math_is_finite(distance):
                    continue

                msg = Float32()
                msg.data = distance

                self.distance_publisher.publish(msg)

            except (ValueError, UnicodeDecodeError):

                self.get_logger().warning(
                    'Received malformed UDP packet.'
                )

    # =================================================
    # MAIN TIMER CALLBACK
    # =================================================

    def timer_callback(self):

        # Send movement command periodically.
        self.send_command()

        # Read all available sensor packets.
        self.receive_sensor_data()

    # =================================================
    # CLEAN SHUTDOWN
    # =================================================

    def destroy_node(self):

        # Send multiple stop packets before shutdown.
        for _ in range(3):

            try:

                self.sock.sendto(
                    b'CMD,0,0',
                    self.esp32_address
                )

            except OSError:
                pass

        self.sock.close()

        super().destroy_node()


def math_is_finite(value):

    return value == value and abs(value) != float('inf')


def main(args=None):

    rclpy.init(args=args)

    node = ESP32Bridge()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

