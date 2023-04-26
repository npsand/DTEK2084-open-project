import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .gradient import gradient


class FormationControl(Node):
    def __init__(self):
        super().__init__('formation_control')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(Odometry, '/drone1/odom', self.odom_callback, qos_profile=qos_policy)
        self.vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)


    def odom_callback(self, msg):
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        goal = [1,1]
        obstacles = [[10,10],[-10,-10]]

        grad = gradient(pos, goal, obstacles)


        vel_msg = Twist()
        vel_msg.linear.x = -grad[1] * 20
        vel_msg.linear.y = grad[0] * 20

        self.vel_pub.publish(vel_msg)



def main(args=None):
    rclpy.init(args=args)
    formation_control = FormationControl()
    rclpy.spin(formation_control)
    formation_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()