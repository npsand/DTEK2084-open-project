import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Vector3, PointStamped, Point
from interfaces.msg import SignalArray
from .gradient import gradient
from rclpy.impl import rcutils_logger


class FormationControl(Node):
    def __init__(self):
        super().__init__('formation_control')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.mavic_1_gps_sub = self.create_subscription(PointStamped, f'/Mavic_2_PRO_1/gps', self.mavic_1_gps_callback, qos_profile=qos_policy)
        self.mavic_2_gps_sub = self.create_subscription(PointStamped, f'/Mavic_2_PRO_2/gps', self.mavic_2_gps_callback, qos_profile=qos_policy)
        self.mavic_3_gps_sub = self.create_subscription(PointStamped, f'/Mavic_2_PRO_3/gps', self.mavic_3_gps_callback, qos_profile=qos_policy)
        self.mavic_4_gps_sub = self.create_subscription(PointStamped, f'/Mavic_2_PRO_4/gps', self.mavic_4_gps_callback, qos_profile=qos_policy)

        self.ground_pose_sub = self.create_subscription(Vector3, '/ground_robot/pose', self.ground_pose_callback, qos_profile=qos_policy)
        self.ground_gps_sub = self.create_subscription(Vector3, '/ground_robot/gps', self.ground_gps_callback, qos_profile=qos_policy)
        
        self.vel_pubs = []
        for i in range(4):
            self.vel_pubs.append(self.create_publisher(Twist, f'/Mavic_2_PRO_{i+1}/cmd_vel', 10))

        
        self.logger = rcutils_logger.RcutilsLogger()

        self.goals = np.array([[1,1],
                              [-1,1],
                              [-1,-1],
                              [1,-1]])
        
        #self.gr_yaw = 0
        
        self.rot_mat = np.array([[np.cos(0), -np.sin(0)],
                                 [np.sin(0), np.cos(0)]])
        
        self.mavic_pos = [[0,0], [0,0], [0,0], [0,0]]



    def mavic_1_gps_callback(self, msg):
        self.mavic_pos[0] = [msg.point.x, msg.point.y]

    def mavic_2_gps_callback(self, msg):
        self.mavic_pos[1] = [msg.point.x, msg.point.y]

    def mavic_3_gps_callback(self, msg):
        self.mavic_pos[2] = [msg.point.x, msg.point.y]

    def mavic_4_gps_callback(self, msg):
        self.mavic_pos[3] = [msg.point.x, msg.point.y]

    def ground_pose_callback(self, msg):
        #self.gr_yaw = msg.z
        self.rot_mat = np.array([[np.cos(msg.z), -np.sin(msg.z)],
                                 [np.sin(msg.z), np.cos(msg.z)]])
        self.logger.info('yaw %g' %msg.z)


    def ground_gps_callback(self, gps_msg):
        relative_goals = []
        for i in range(4):
            goal = np.array([self.goals[i][0], self.goals[i][1]])
            rot_goal = self.rot_mat @ goal

            rel_x = gps_msg.x + rot_goal[0]
            rel_y = gps_msg.y + rot_goal[1]

            rel = np.array([rel_x, rel_y])
            if i == 3:
                self.logger.info('3 goal %s' % rot_goal)


            relative_goals.append(rel)

        for i, goal in enumerate(relative_goals):
            grad = gradient(self.mavic_pos[i], goal, [[1000,1000],[-1000,-1000]])
            vel_msg = Twist()
            vel_msg.linear.x = -grad[0] * 500
            vel_msg.linear.y = -grad[1] * 500

            self.vel_pubs[i].publish(vel_msg)

        


    """
    for i in range(4):
        signal = msg.signals[i]

        pos = self.calc_pos_from_signal(signal)
        self.logger.info('x %g, y %g, i %d' % (pos[0], pos[1], i))

        rot = np.array([[0, 1],
                        [-1, 0]])
        rot_mat = np.array([[np.cos(self.gr_yaw), -np.sin(self.gr_yaw)],
                        [np.sin(self.gr_yaw), np.cos(self.gr_yaw)]])
        #goal = rot @ goal
        #goal = rot_mat @ goal

        obstacles = [[100,100],[-100,-100]]

        grad = gradient(pos, self.goals[i], obstacles)
        #grad = rot @ grad
        grad = rot_mat @ grad


        vel_msg = Twist()
        vel_msg.linear.x = -grad[0] * 1000
        vel_msg.linear.y = -grad[1] * 1000

        self.vel_pubs[i].publish(vel_msg)

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
        #projected = rot @ projected

        return [projected[0], projected[1]]
    """





def main(args=None):
    rclpy.init(args=args)
    formation_control = FormationControl()
    rclpy.spin(formation_control)
    formation_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()