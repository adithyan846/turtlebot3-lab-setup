
#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# =====================================================
# ROBOT BEHAVIOUR CONFIGURATION
# =====================================================

OBSTACLE_DISTANCE_CM = 25.0

FORWARD_SPEED_MPS = 0.15

TURN_SPEED_RPS = 0.70

SENSOR_TIMEOUT = 0.5

CONTROL_RATE_HZ = 10.0


class ObstacleAvoider(Node):

    def __init__(self):

        super().__init__('obstacle_avoider')

        # -------------------------------------------------
        # SENSOR STATE
        # -------------------------------------------------

        self.distance_cm = None
        self.last_sensor_time = None

        # -------------------------------------------------
        # SENSOR SUBSCRIBER
        # -------------------------------------------------

        self.sensor_subscription = self.create_subscription(
            Float32,
            '/ultrasonic/distance_cm',
            self.sensor_callback,
            10
        )

        # -------------------------------------------------
        # MOVEMENT PUBLISHER
        # -------------------------------------------------

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # -------------------------------------------------
        # CONTROL TIMER
        # -------------------------------------------------

        self.timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ,
            self.control_callback
        )

        self.get_logger().info(
            'Obstacle avoidance controller started.'
        )

    # =================================================
    # SENSOR CALLBACK
    # =================================================

    def sensor_callback(self, msg):

        self.distance_cm = msg.data

        self.last_sensor_time = time.monotonic()

    # =================================================
    # CHECK SENSOR VALIDITY
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
    # MAIN DECISION-MAKING LOGIC
    # =================================================

    def control_callback(self):

        # Stop if sensor data is missing or stale.
        if not self.sensor_is_valid():

            self.stop_robot()

            self.get_logger().warning(
                'Invalid or stale sensor data. Stopping.'
            )

            return

        distance = self.distance_cm

        self.get_logger().info(
            f'Distance: {distance:.1f} cm'
        )

        # Obstacle detected
        if distance <= OBSTACLE_DISTANCE_CM:

            self.get_logger().info(
                'Obstacle detected. Turning.'
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

