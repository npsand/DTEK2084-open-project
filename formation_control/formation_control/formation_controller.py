import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from interfaces.msg import SignalArray
from .gradient import gradient


class FormationControl(Node):
    def __init__(self):
        super().__init__('formation_control')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(SignalArray, '/ground_robot/signals', self.signal_callback, qos_profile=qos_policy)
        self.vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)


    def signal_callback(self, msg):
        signal = msg.signals[0]

        pos = self.calc_pos_from_signal(signal)

        goal = [1,1]
        obstacles = [[10,10],[-10,-10]]

        grad = gradient(pos, goal, obstacles)


        vel_msg = Twist()
        vel_msg.linear.x = -grad[0] * 20
        vel_msg.linear.y = -grad[1] * 20

        self.vel_pub.publish(vel_msg)

    def calc_pos_from_signal(self, signal):
        str = signal.signal_strength
        r = np.sqrt(1/str)
        x = signal.signal_dir_x * r
        y = signal.signal_dir_y * r
        z = signal.signal_dir_z * r

        vec_1 = np.array([x,y,z])
        vec_2 = np.array([x,y,0])

        projected = vec_2 * np.dot(vec_2, vec_1) / np.dot(vec_2, vec_2)

        return [projected[0], projected[1]]





def main(args=None):
    rclpy.init(args=args)
    formation_control = FormationControl()
    rclpy.spin(formation_control)
    formation_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()