"""
decisions.py

This module implements the decision-making node for robot navigation in ROS2.
It integrates localization, planning, and control to guide the robot towards
a goal point or along a trajectory, publishing velocity commands.
"""

# Imports
import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

class decision_maker(Node):
    """
    ROS2 node responsible for decision-making in robot navigation.

    This node subscribes to localization data, plans paths, computes control
    commands, and publishes velocity to move the robot towards the goal.
    """

    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):
        """
        Initializes the decision_maker node.

        Args:
            publisher_msg: Message type for publishing (e.g., Twist).
            publishing_topic (str): Topic to publish velocity commands.
            qos_publisher: QoS profile for the publisher.
            goalPoint: Goal point(s) for planning.
            rate (int): Publishing rate in Hz.
            motion_type: Type of motion planner (POINT_PLANNER or TRAJECTORY_PLANNER).
        """
        super().__init__("decision_maker")

        # TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.vel_publisher = self.create_publisher(publisher_msg, publishing_topic, qos_profile=qos_publisher)

        # Calculate publishing period from rate
        publishing_period = 1 / rate
        # Deadzone thresholds for goal detection
        self.linear_deadzone = 0.05 
        self.time_in_zone = 0.5 #500 ms
        self.enterance_time = False

        # Instantiate the controller based on motion type
        # TODO Part 5: Tune your parameters here
        if motion_type == POINT_PLANNER:
            self.controller = controller(klp=0.4, klv=0.02, kli=1.0, kap=0.8, kav=0.6)
            self.planner = planner(POINT_PLANNER)

        elif motion_type == TRAJECTORY_PLANNER:
            self.controller = trajectoryController(klp=0.4, klv=0.02, kli=1.0, kap=0.8, kav=0.6)
            self.planner = planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)

        # Instantiate the localization, use rawSensor for now
        self.localizer = localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal = self.planner.plan(goalPoint)
        if motion_type == POINT_PLANNER:
            self.end_goal = self.goal
        elif motion_type == TRAJECTORY_PLANNER:
            self.end_goal = self.goal[-1]

        # Create a timer for periodic callback
        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        """
        Timer callback function executed periodically.

        Updates localization, checks if goal is reached, computes control
        commands, and publishes velocity.
        """
        # TODO Part 3: Run the localization node
        spin_once(self.localizer)
        # Remember that this file is already running the decision_maker node.

        # Get current pose from localizer
        pose = self.localizer.getPose()

        if pose is None:
            print("waiting for odom msgs ....")
            return

        # TODO Part 3: Check if you reached the goal
        # Check if both angular and linear errors are within deadzones
        if calculate_linear_error(current_pose=pose, goal_pose=self.end_goal) <= self.linear_deadzone:
            if self.enterance_time is False:
                # Record the time when first entering the deadzone
                self.enterance_time_seconds = Time.from_msg(pose[3]).nanoseconds / 1e9
                self.enterance_time = True
            elif self.enterance_time is True and ((Time.from_msg(pose[3]).nanoseconds / 1e9) - self.enterance_time_seconds) > self.time_in_zone:
                # Confirm goal reached after staying in deadzone for 0.1 seconds
                self.reached_goal = True
            # self.reached_goal = True
        else:
            self.reached_goal = False
            self.enterance_time = False

        # Create velocity command message
        cmd_vel_msg = Twist()

        if self.reached_goal:
            # Publish zero velocity to stop the robot
            self.vel_publisher.publish(cmd_vel_msg)

            # Save logs for PID controllers
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()

            print("reached goal")
            raise SystemExit

        # Request velocity commands from controller
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        # TODO Part 4: Publish the velocity to move the robot
        cmd_vel_msg.linear.x = velocity
        cmd_vel_msg.angular.z = yaw_rate

        self.vel_publisher.publish(cmd_vel_msg)

import argparse


def main(args=None):
    """
    Main function to initialize and run the decision_maker node.

    Sets up QoS profiles, instantiates the decision_maker based on motion type,
    and spins the node until goal is reached.

    Args:
        args: Parsed command-line arguments.
    """
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

    # QoS profiles for velocity topics
    vel_qos = QoSProfile(reliability=2, durability=2, history=1, depth=10)

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM = decision_maker(publisher_msg=Twist, publishing_topic="/cmd_vel", qos_publisher=vel_qos, goalPoint=[-1.0, -1.0], rate=10, motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM = decision_maker(publisher_msg=Twist, publishing_topic="/cmd_vel", qos_publisher=vel_qos, goalPoint=None, rate=10, motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)

    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__ == "__main__":
    # Parse command-line arguments
    argParser = argparse.ArgumentParser(description="point or trajectory")
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
