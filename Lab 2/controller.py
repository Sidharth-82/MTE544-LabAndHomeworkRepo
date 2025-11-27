"""
controller.py

This module implements PID controllers for robot velocity control,
including linear and angular velocity regulation with saturation limits.
"""

import numpy as np

from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI = 3.1415926535

P = 0
PD = 1
PI = 2
PID = 3


class controller:
    """
    PID controller for point-to-point navigation.

    Manages linear and angular velocity control using PID algorithms.
    """

    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        """
        Initializes the controller with PID gains and velocity limits.

        Args:
            klp (float): Linear proportional gain.
            klv (float): Linear derivative gain.
            kli (float): Linear integral gain.
            kap (float): Angular proportional gain.
            kav (float): Angular derivative gain.
            kai (float): Angular integral gain.
        """
        self.max_linear_vel = 0.31
        self.max_angular_vel = 1.90

        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        lin_controller_mode = PID
        ang_controller_mode = PID

        self.PID_linear = PID_ctrl(lin_controller_mode, klp, klv, kli, filename_="linear_errors.csv")
        self.PID_angular = PID_ctrl(ang_controller_mode, kap, kav, kai, filename_="angular_errors.csv")

    def vel_request(self, pose, goal, status):
        """
        Computes velocity commands based on current pose and goal.

        Args:
            pose (tuple): Current pose (x, y, theta, time).
            goal (tuple): Goal pose (x, y).
            status (bool): Whether to update PID or return 0.

        Returns:
            tuple: (linear_velocity, angular_velocity) with saturation.
        """
        # Calculate errors
        e_lin = calculate_linear_error(pose, goal)
        e_ang = calculate_angular_error(pose, goal)

        # Update PID controllers
        linear_vel = self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel = self.PID_angular.update([e_ang, pose[3]], status)

        # TODO Part 4: Add saturation limits for the robot linear and angular velocity (hint: you can use np.clip function)
        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)

        return linear_vel, angular_vel


class trajectoryController(controller):
    """
    PID controller for trajectory following.

    Extends controller to handle multiple waypoints in a trajectory.
    """

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        """
        Initializes the trajectory controller.

        Args:
            klp (float): Linear proportional gain.
            klv (float): Linear derivative gain.
            kli (float): Linear integral gain.
            kap (float): Angular proportional gain.
            kav (float): Angular derivative gain.
            kai (float): Angular integral gain.
        """
        super().__init__(klp, klv, kli, kap, kav, kai)

    def vel_request(self, pose, listGoals, status):
        """
        Computes velocity for trajectory following.

        Args:
            pose (tuple): Current pose (x, y, theta, time).
            listGoals (list): List of goal points in trajectory.
            status (bool): Whether to update PID or return 0.

        Returns:
            tuple: (linear_velocity, angular_velocity) with saturation.
        """
        # Select intermediate goal ahead of current position
        goal = self.lookFarFor(pose, listGoals)

        finalGoal = listGoals[-1]

        # Calculate errors to final goal and intermediate goal
        e_lin = calculate_linear_error(pose, finalGoal)
        e_ang = calculate_angular_error(pose, goal)

        # Update PID controllers
        linear_vel = self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel = self.PID_angular.update([e_ang, pose[3]], status)

        # TODO Part 5: Add saturation limits for the robot linear and angular velocity (hint: you can use np.clip function)
        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)

        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        """
        Finds the best intermediate goal point ahead in the trajectory.

        Args:
            pose (tuple): Current pose (x, y, theta, time).
            listGoals (list): List of [x, y] goal points.

        Returns:
            list: Selected goal point [x, y].
        """
        # Convert to numpy arrays for vectorized operations
        poseArray = np.array([pose[0], pose[1]])
        listGoalsArray = np.array(listGoals)

        # Calculate squared distances
        distanceSquared = np.sum((listGoalsArray - poseArray)**2, axis=1)
        closestIndex = np.argmin(distanceSquared)

        # Select goal 3 points ahead or last point if near end
        return listGoals[min(closestIndex + 3, len(listGoals) - 1)]
