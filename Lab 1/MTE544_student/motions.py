# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    # ROS2 node for executing different motion types and logging sensor data

    def __init__(self, motion_type=0):
        # Initialize the node with motion type
        super().__init__("motion_types")

        self.type=motion_type

        self.radius_=0.25  # Default radius for circular/spiral motion
        self.linear_speed=0.0  # Initial linear speed for acceleration

        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False

        self.latest_odom = None

        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", 10)

        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.imu_sub_=self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=qos)
        ...

        # ENCODER subscription
        self.encoder_sub_=self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=qos)
        ...

        # LaserScan subscription
        self.laserscan_sub_=self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile=qos)
        ...

        self.create_timer(0.1, self.timer_callback)



    def imu_callback(self, imu_msg: Imu):
        # Callback for IMU sensor data: log accelerations and angular velocity
        if not self.imu_initialized:
            self.imu_initialized = True

        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds
        quat = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        _, _, yaw = euler_from_quaternion(quat=quat)
        ang_vel_z = imu_msg.angular_velocity.z
        lin_accel_x = imu_msg.linear_acceleration.x
        lin_accel_y = imu_msg.linear_acceleration.y
        values_list = [lin_accel_x,lin_accel_y, ang_vel_z, timestamp]
        self.imu_logger.log_values(values_list)


    def odom_callback(self, odom_msg: Odometry):
        # Callback for odometry data: log velocities and yaw angle
        if not self.odom_initialized:
            self.odom_initialized = True

        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds
        self.latest_odom = odom_msg
        # lin_x = odom_msg.twist.twist.linear.x
        # lin_y=odom_msg.twist.twist.linear.y
        
        # yaw = odom_msg.twist.twist.angular.z
        
        #supposed to be for position
        pos_x = odom_msg.pose.pose.position.x
        pos_y = odom_msg.pose.pose.position.y
        
        _, _, pos_yaw = euler_from_quaternion(odom_msg.pose.pose.orientation)
        

        values_list = [pos_x, pos_y, pos_yaw, timestamp]
        self.odom_logger.log_values(values_list)


    def laser_callback(self, laser_msg: LaserScan):
        # Callback for laser scan data: log ranges and angle increment
        if not self.laser_initialized:
            self.laser_initialized = True

        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        angle_increment = laser_msg.angle_increment
        ranges = laser_msg.ranges
        values_list = [ranges, angle_increment, timestamp]
        self.laser_logger.log_values(values_list)

    def timer_callback(self):
        # Timer callback: check initialization and publish velocity commands based on motion type
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True

        if not self.successful_init:
            return

        cmd_vel_msg=Twist()

        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()

        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()

        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()

        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit

        self.vel_publisher.publish(cmd_vel_msg)
        
    

    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        # Generate Twist for circular motion: constant linear velocity, angular velocity = linear / radius
        msg=Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5 / self.radius_
        return msg

    def make_spiral_twist(self):
        # Generate Twist for spiral motion: increasing linear velocity, constant angular velocity
        msg=Twist()
        self.linear_speed += 0.01
        msg.linear.x = self.linear_speed
        msg.angular.z = 0.5 / self.radius_
        return msg

    def make_acc_line_twist(self):
        # Generate Twist for accelerated line motion: increasing linear velocity, no angular velocity
        msg=Twist()
        self.linear_speed += 0.01
        msg.linear.x = self.linear_speed
        msg.angular.z = 0.0
        return msg

import argparse

if __name__=="__main__":
    # Main entry point: parse motion type argument and run the ROS2 node

    argParser=argparse.ArgumentParser(description="input the motion type")

    argParser.add_argument("--motion", type=str, default="circle")

    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":
        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)
    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)
    else:
        print(f"we don't have {args.motion.lower()} motion type")

    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
