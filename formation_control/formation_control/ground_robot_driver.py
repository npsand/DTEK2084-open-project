import rclpy
from rclpy.impl import rcutils_logger

from geometry_msgs.msg import Twist
from interfaces.msg import Signal
from interfaces.msg import SignalArray

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class GroundRobotDriver:  
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.logger = rcutils_logger.RcutilsLogger()


        self.receiver = self.__robot.getDevice('ground_receiver')
        self.receiver.enable(self.__timestep)
        self.receiver.setChannel(1)

        #self.__left_motor = self.__robot.getDevice('left wheel motor')
        #self.__right_motor = self.__robot.getDevice('right wheel motor')

        #self.__left_motor.setPosition(float('inf'))
        #self.__left_motor.setVelocity(0)

        #self.__right_motor.setPosition(float('inf'))
        #self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('ground_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.signal_pub = self.__node.create_publisher(SignalArray, 'ground_robot/signals', 10)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        arr = []
        signal_arr_msg = SignalArray()

        if self.receiver.getQueueLength() > 0:
            signal_msg = Signal()
            signal_msg.signal_strength = self.receiver.getSignalStrength()
            direction_em = self.receiver.getEmitterDirection()
            signal_msg.signal_dir_x = direction_em[0]
            signal_msg.signal_dir_y = direction_em[1]
            signal_msg.signal_dir_z = direction_em[2]

            arr.append(signal_msg)
            signal_arr_msg.signals = arr
            self.signal_pub.publish(signal_arr_msg)
        

        while self.receiver.getQueueLength() > 0:
            self.receiver.nextPacket()

        #forward_speed = self.__target_twist.linear.x
        #angular_speed = self.__target_twist.angular.z

        #command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        #command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        #self.__left_motor.setVelocity(command_motor_left)
        #self.__right_motor.setVelocity(command_motor_right)