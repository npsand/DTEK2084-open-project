import rclpy
from rclpy.impl import rcutils_logger

from geometry_msgs.msg import Twist, Vector3
from interfaces.msg import Signal, SignalArray

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class GroundRobotDriver:  
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.logger = rcutils_logger.RcutilsLogger()


        self.receiver = self.__robot.getDevice('ground_receiver')
        self.receiver.enable(self.__timestep)

        self.channel = 1
        self.receiver.setChannel(1)
        self.signal_arr = []

        self.imu = self.__robot.getDevice('inertial_unit')
        self.imu.enable(self.__timestep)

        self.gps = self.__robot.getDevice('gps')
        self.gps.enable(self.__timestep)

        self.wheels = []

        for i in range(4):
            self.wheels.append(self.__robot.getDevice(f'wheel{i+1}'))
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(0.0)


        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('ground_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.signal_pub = self.__node.create_publisher(SignalArray, 'ground_robot/signals', 10)
        self.pose_pub = self.__node.create_publisher(Vector3, 'ground_robot/pose', 10)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        turn_left = 1
        turn_right = 0.9

        speed = 2

        self.wheels[0].setVelocity(speed * turn_right)
        self.wheels[1].setVelocity(speed * turn_left)
        self.wheels[2].setVelocity(speed * turn_right)
        self.wheels[3].setVelocity(speed * turn_left)


        if self.receiver.getQueueLength() > 0:
            signal_msg = Signal()
            signal_msg.signal_strength = self.receiver.getSignalStrength()
            direction_em = self.receiver.getEmitterDirection()
            signal_msg.signal_dir_x = direction_em[0]
            signal_msg.signal_dir_y = direction_em[1]
            signal_msg.signal_dir_z = direction_em[2]

            self.signal_arr.append(signal_msg)

            self.receiver.nextPacket()

        self.channel += 1
        self.receiver.setChannel(self.channel)

        if self.channel > 4:
            signal_arr_msg = SignalArray()
            signal_arr_msg.signals = self.signal_arr
            self.signal_pub.publish(signal_arr_msg)
            self.channel = 1
            self.receiver.setChannel(self.channel)
            self.signal_arr = []


        pose_msg = Vector3()
        rpy = self.imu.getRollPitchYaw()
        pose_msg.x = rpy[0]
        pose_msg.y = rpy[1]
        pose_msg.z = rpy[2]
        self.pose_pub.publish(pose_msg)