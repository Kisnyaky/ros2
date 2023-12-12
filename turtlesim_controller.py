import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(
                            Twist, '/turtle1/cmd_vel', 10)
        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10)

    # New method for TurtlesimController
    def cb_pose(self, msg):
        self.pose = msg

    def go_to(self, speed, omega, x, y):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.pose is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        # Stuff with atan2
        x0 = self.pose.x
        y0 = self.pose.y
        theta_0 = math.degrees(self.pose.theta)

        theta_1 = math.degrees(math.atan2(y-y0, x-x0))
        angle = theta_1 - theta_0
        distance = math.sqrt(((x - x0) * (x - x0)) + (y - y0) * (y - y0))

        # Execute movement
        self.turn(omega, angle)
        self.go_straight(speed, distance)




    def go_straight(self, speed, distance):
        # Implement straght motion here
        # Create and publish msg
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(distance / speed)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() <= when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')


    def turn(self, omega, angle):
        # Implement straght motion here
        # Create and publish msg
        vel_msg = Twist()

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        if angle > 0:
            vel_msg.angular.z = math.radians(omega)
        else:
            vel_msg.angular.z = -math.radians(omega)


        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(angle / omega)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() <= when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)
        #self.get_logger().info('Arrived to destination.')

    def draw_poly(self, speed, omega, N, a):

        angle = 360.0 / N
        for i in range(N):
            self.go_straight(speed, a)
            self.turn(omega, angle)
            
    def koch_gorbe(self, N):
        iteraciok = 3
        if N >= 2:
            for i in range(N-1):
                iteraciok = iteraciok * 4
        s = 1.0
        o = 100.0
        a = 120.0
        d = 3
        dis = 3
        for i in range(N):
            dis = dis / d
        if N == 0:
            for i in range(iteraciok):
                self.go_straight(s, d)
                self.turn(o, a)
        else:
            if N == 1:
                for i in range(iteraciok):
                    self.go_straight(s, dis)
                    self.turn(o, -a/2)
                    self.go_straight(s, dis)
                    self.turn(o, a)
                    self.go_straight(s, dis)
                    self.turn(o, -a/2)
                    self.go_straight(s, dis)
                    self.turn(o, a)
            else:
                for i in range(iteraciok):
                    self.go_straight(s, dis)
                    self.turn(o, -a/2)
                    self.go_straight(s, dis)
                    self.turn(o, a)
                    self.go_straight(s, dis)
                    self.turn(o, -a/2)
                    self.go_straight(s, dis)
                    if i%2 == 0:
                        self.turn(o, -a/2)
                    else:
                        self.turn(o, a)
            


def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()
    #tc.go_straight(1.0, 4.0)
    #tc.draw_poly(1.0, 36.0, 6, 2.0)

    #tc.go_to(1.0, 20.0, 2, 8)
    #tc.go_to(1.0, 20.0, 2, 2)
    #tc.go_to(1.0, 20.0, 3, 4)
    #tc.go_to(1.0, 20.0, 6, 2)

    tc.koch_gorbe(2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
