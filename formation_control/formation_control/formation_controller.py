import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from interfaces.msg import SignalArray
from .gradient import gradient
from rclpy.impl import rcutils_logger


class FormationControl(Node):
    def __init__(self):
        super().__init__('formation_control')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.signal_sub = self.create_subscription(SignalArray, '/ground_robot/signals', self.signal_callback, qos_profile=qos_policy)
        self.pose_sub = self.create_subscription(Vector3, '/ground_robot/pose', self.pose_callback, qos_profile=qos_policy)
        self.vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.logger = rcutils_logger.RcutilsLogger()

        self.gr_yaw = 0

    def pose_callback(self, msg):
        self.gr_yaw = msg.z
        
        self.logger.info('yaw %g' % self.gr_yaw)


    def signal_callback(self, msg):
        signal = msg.signals[0]

        pos = self.calc_pos_from_signal(signal)
        self.logger.info('x %g, y %g' % (pos[0], pos[1]))

        goal = np.array([1,1])

        rot = np.array([[0, 1],
                        [-1, 0]])
        rot_mat = np.array([[np.cos(self.gr_yaw), -np.sin(self.gr_yaw)],
                           [np.sin(self.gr_yaw), np.cos(self.gr_yaw)]])
        #goal = rot @ goal
        #goal = rot_mat @ goal

        obstacles = [[100,100],[-100,-100]]

        grad = gradient(pos, goal, obstacles)
        grad = rot @ grad
        grad = rot_mat @ grad


        vel_msg = Twist()
        vel_msg.linear.x = -grad[0] * 1000
        vel_msg.linear.y = -grad[1] * 1000

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
        projected = projected[:2]

        rot = np.array([[0, -1],
                        [1, 0]])
        projected = rot @ projected

        return [projected[0], projected[1]]





def main(args=None):
    rclpy.init(args=args)
    formation_control = FormationControl()
    rclpy.spin(formation_control)
    formation_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()