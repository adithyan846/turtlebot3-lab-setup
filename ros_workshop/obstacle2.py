
#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# =====================================================
# ROBOT CONFIGURATION
# =====================================================

OBSTACLE_DISTANCE_CM = 25.0

FORWARD_SPEED_MPS = 0.15
TURN_SPEED_RPS = 0.70

SENSOR_TIMEOUT = 1.5

CONTROL_RATE_HZ = 10.0

# Maximum ultrasonic sensor reading
MAX_SENSOR_DISTANCE_CM = 400.0


class ObstacleAvoider(Node):

    def __init__(self):

        super().__init__('obstacle_avoider')

        self.distance_cm = None
        self.last_sensor_time = None

        self.sensor_subscription = self.create_subscription(
            Float32,
            '/ultrasonic/distance_cm',
            self.sensor_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ,
            self.control_callback
        )

        self.sensor_warning_printed = False

        self.get_logger().info(
            'Obstacle avoidance controller started.'
        )

    # =================================================
    # SENSOR CALLBACK
    # =================================================

    def sensor_callback(self, msg):

        self.distance_cm = msg.data

        self.last_sensor_time = time.monotonic()

        # Print every received sensor reading
        self.get_logger().info(
            f'[SENSOR] Distance: {self.distance_cm:.2f} cm'
        )

    # =================================================
    # SENSOR VALIDITY
    # =================================================

    def sensor_is_valid(self):

        if self.distance_cm is None:
            return False

        if self.last_sensor_time is None:
            return False

        age = time.monotonic() - self.last_sensor_time

        if age > SENSOR_TIMEOUT:
            return False

        if self.distance_cm <= 0:
            return False

        return True

    # =================================================
    # MOVEMENT COMMANDS
    # =================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

    def move_forward(self):

        cmd = Twist()

        cmd.linear.x = FORWARD_SPEED_MPS
        cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)

    def turn_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = TURN_SPEED_RPS

        self.cmd_publisher.publish(cmd)

    # =================================================
    # MAIN CONTROL LOGIC
    # =================================================

    def control_callback(self):

        # Stop if sensor data is missing or stale
        if not self.sensor_is_valid():

            self.stop_robot()

            if not self.sensor_warning_printed:

                self.get_logger().warning(
                    'Sensor data missing or stale. Robot stopped.'
                )

                self.sensor_warning_printed = True

            return

        self.sensor_warning_printed = False

        distance = self.distance_cm

        # Maximum range means clear path
        if distance >= MAX_SENSOR_DISTANCE_CM:

            self.get_logger().info(
                '[CLEAR] No obstacle within sensor range.'
            )

            self.move_forward()

        # Obstacle detected
        elif distance <= OBSTACLE_DISTANCE_CM:

            self.get_logger().info(
                '[OBSTACLE] Obstacle detected. Turning.'
            )

            self.turn_robot()

        # Clear path
        else:

            self.move_forward()


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoider()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()

