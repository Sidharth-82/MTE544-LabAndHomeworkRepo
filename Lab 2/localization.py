"""
localization.py

This module implements localization for robot pose estimation using
odometry data from ROS2 topics, with logging capabilities.
"""

import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0


class localization(Node):
    """
    ROS2 node for robot localization using odometry data.

    Subscribes to /odom topic, processes pose information, and logs
    position and orientation data.
    """

    def __init__(self, localizationType=rawSensor):
        """
        Initializes the localization node.

        Args:
            localizationType (int): Type of localization sensor (currently only rawSensor).
        """
        super().__init__("localizer")

        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        odom_qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # Logger for robot pose data
        self.loc_logger = Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose = None

        if localizationType == rawSensor:
            # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.encoder_sub_ = self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        else:
            print("This type doesn't exist", file=sys.stderr)

    def odom_callback(self, pose_msg):
        """
        Callback for odometry messages.

        Extracts pose information and logs it.

        Args:
            pose_msg (Odometry): ROS2 Odometry message.
        """
        # TODO Part 3: Read x,y, theta, and record the stamp
        self.pose = (pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y,
                     euler_from_quaternion(pose_msg.pose.pose.orientation), pose_msg.header.stamp)

        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2],
                                   Time.from_msg(self.pose[3]).nanoseconds / 1e9])

    def getPose(self):
        """
        Returns the current pose.

        Returns:
            tuple: (x, y, theta, timestamp) or None if not available.
        """
        return self.pose


# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py

def main(args=None):
    """
    Main function to run the localization node standalone.

    Args:
        args: Command-line arguments (unused).
    """
    init()
    Loc = localization(localizationType=rawSensor)
    spin(Loc)


if __name__ == "__main__":
    main()
