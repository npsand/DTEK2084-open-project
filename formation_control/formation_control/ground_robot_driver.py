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
        # Distance sensors
        self.ds = []
        dsNames = ['ds_right', 'ds_left']
        for i in range(2):
            self.ds.append(self.__robot.getDevice(dsNames[i]))
            self.ds[i].enable(self.__timestep)

        self.avoidObstacleCounter = 0


        self.wheels = []

        for i in range(4):
            self.wheels.append(self.__robot.getDevice(f'wheel{i+1}'))
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(0.0)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('ground_robot_driver')
        self.signal_pub = self.__node.create_publisher(SignalArray, 'ground_robot/signals', 10)
        self.pose_pub = self.__node.create_publisher(Vector3, 'ground_robot/pose', 10)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        # Obstacle avoidance
        leftSpeed = 2.1
        rightSpeed = 2.0
        if self.avoidObstacleCounter > 0:
            self.avoidObstacleCounter -= 1
            leftSpeed = 0.3
            rightSpeed = -0.3
        else:  # read sensors
            for i in range(2):
                if self.ds[i].getValue() < 950.0:
                    self.avoidObstacleCounter = 100
        
        self.wheels[0].setVelocity(leftSpeed)
        self.wheels[1].setVelocity(rightSpeed)
        self.wheels[2].setVelocity(leftSpeed)
        self.wheels[3].setVelocity(rightSpeed)

        # Get signal strengths and directions from drone
        if self.receiver.getQueueLength() > 0:
            signal_msg = Signal()
            signal_msg.signal_strength = self.receiver.getSignalStrength()
            direction_em = self.receiver.getEmitterDirection()
            signal_msg.signal_dir_x = direction_em[0]
            signal_msg.signal_dir_y = direction_em[1]
            signal_msg.signal_dir_z = direction_em[2]

            self.signal_arr.append(signal_msg)

            self.receiver.nextPacket()

        # Change channel to next drone's channel
        self.channel += 1
        self.receiver.setChannel(self.channel)

        # Publish drone signals when all channels have been visited
        if self.channel > 4:
            signal_arr_msg = SignalArray()
            signal_arr_msg.signals = self.signal_arr
            self.signal_pub.publish(signal_arr_msg)
            self.channel = 1
            self.receiver.setChannel(self.channel)
            self.signal_arr = []

        # Publish pose
        pose_msg = Vector3()
        rpy = self.imu.getRollPitchYaw()
        pose_msg.x = rpy[0]
        pose_msg.y = rpy[1]
        pose_msg.z = rpy[2]
        self.pose_pub.publish(pose_msg)