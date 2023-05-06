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
        self.pubs = []
        for i in range(4):
            self.pubs.append(self.create_publisher(Twist, f'/Mavic_2_PRO_{i+1}/cmd_vel', 10))
        self.logger = rcutils_logger.RcutilsLogger()

        self.goals = np.array([[-1,1],
                              [-1,-1],
                              [1,-1],
                              [1,1]])
        
        self.rot_cw = np.array([[0, 1],
                                [-1, 0]])
        self.rot_ccw = np.array([[0, -1],
                                 [1, 0]])

        self.yaw_offset_mat = np.array([[np.cos(0), -np.sin(0)],
                                        [np.sin(0), np.cos(0)]])

        

    def pose_callback(self, msg):
        self.yaw_offset_mat = np.array([[np.cos(msg.z), -np.sin(msg.z)],
                                        [np.sin(msg.z), np.cos(msg.z)]])
        


    def signal_callback(self, msg):

        pos_arr = []

        for i in range(len(msg.signals)):
            signal = msg.signals[i]

            pos_arr.append(self.calc_pos_from_signal(signal))


        for i in range(len(msg.signals)):
            obstacles = pos_arr.copy()
            obstacles.pop(i)
            self.logger.info('%s, i %d' % (obstacles, i))
            self.logger.info('pos %s' % pos_arr[i])
            self.logger.info('goal %s' % self.goals[i])

            grad = gradient(pos_arr[i], self.goals[i], obstacles)
            grad = self.rot_cw @ grad
            grad = self.yaw_offset_mat @ grad

            vel_msg = Twist()
            vel_msg.linear.x = -grad[0]
            vel_msg.linear.y = -grad[1]

            # Speed limit to avoid crashing
            if abs(vel_msg.linear.x) > 1.5:
                vel_msg.linear.x = np.sign(vel_msg.linear.x) * 1.5
            if abs(vel_msg.linear.y) > 1.5:
                vel_msg.linear.y = np.sign(vel_msg.linear.y) * 1.5

            self.pubs[i].publish(vel_msg)

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
        projected = self.rot_ccw @ projected

        return [projected[0], projected[1]]





def main(args=None):
    rclpy.init(args=args)
    formation_control = FormationControl()
    rclpy.spin(formation_control)
    formation_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()