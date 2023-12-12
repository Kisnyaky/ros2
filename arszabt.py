import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class ProportionalController(Node):
    def __init__(self):
        super().__init__('proportional_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.subscription_

        self.desired_x = 10.0  # Kívánt x-pozíció
        self.kp = 1.0  # Arányossági tényező

        self.twist_msg = Twist()

    def pose_callback(self, msg):
        # Számold ki a kimeneti hibát (a kívánt és a tényleges x-pozíció között)
        error = self.desired_x - msg.x

        # Számold ki a vezérlőjelet az arányossági tényezővel
        control_signal = self.kp * error

        # Állítsd be a tengeri teknős sebességét a számolt vezérlőjellel
        self.twist_msg.linear.x = control_signal
        self.twist_msg.angular.z = 0.0

        # Küldd el a sebességüzenetet
        self.publisher_.publish(self.twist_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = ProportionalController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

