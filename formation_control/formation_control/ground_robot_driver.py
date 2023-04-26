import rclpy
from geometry_msgs.msg import Twist
from rclpy.impl import rcutils_logger

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

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)


        self.receiver.enable(self.__timestep)
        self.logger.info('signal strength %f' % self.receiver.getSignalStrength())

        if self.receiver.getQueueLength() > 0:
            direction_em = self.receiver.getEmitterDirection()
            self.logger.info('direction_em %s' % direction_em)

        while self.receiver.getQueueLength() > 0:
            packet = self.receiver.getData()
            #self.logger.info('packet %s' % packet)
            self.receiver.nextPacket()

        #forward_speed = self.__target_twist.linear.x
        #angular_speed = self.__target_twist.angular.z

        #command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        #command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        #self.__left_motor.setVelocity(command_motor_left)
        #self.__right_motor.setVelocity(command_motor_right)