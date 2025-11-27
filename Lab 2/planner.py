"""
planner.py

This module defines planners for robot navigation, including point planning
and trajectory planning with predefined paths like parabolas and sigmoids.
"""

# Type of planner
import math

POINT_PLANNER = 0
TRAJECTORY_PLANNER = 1


class planner:
    """
    Planner class for generating navigation paths.

    Supports point-to-point planning and trajectory planning with
    predefined shapes.
    """

    def __init__(self, type_):
        """
        Initializes the planner with a specific type.

        Args:
            type_ (int): Type of planner (POINT_PLANNER or TRAJECTORY_PLANNER).
        """
        self.type = type_

    def plan(self, goalPoint=[-1.0, -1.0]):
        """
        Plans the path based on the planner type.

        Args:
            goalPoint (list): List of goal points.

        Returns:
            tuple or list: Planned path.
        """
        if self.type == POINT_PLANNER:
            return self.point_planner(goalPoint)

        elif self.type == TRAJECTORY_PLANNER:
            return self.trajectory_planner()

    def point_planner(self, goalPoint):
        """
        Simple point planner that returns the goal point.

        Args:
            goalPoint (list): [x, y] coordinates of the goal.

        Returns:
            tuple: (x, y) of the goal.
        """
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, goalPointList):
        """
        Generates a trajectory path using a predefined shape.

        Currently uses sigmoid generation.

        Args:
            goalPointList: Not used in current implementation.

        Returns:
            list: List of [x, y] points defining the trajectory.
        """
        # Define a simple trajectory: a sigmoid path
        ret_val = self.sigmoid_gen(0.0, 2.5, 0.1)
        # ret_val = self.parabolla_gen(0.0, 1.5, 0.1)
        print(ret_val)
        return ret_val

    def parabolla_gen(self, x_min, x_max, step):
        """
        Generates points along a parabolic curve y = x^2.

        Args:
            x_min (float): Starting x value.
            x_max (float): Ending x value.
            step (float): Step size for x.

        Returns:
            list: List of [x, y] points.
        """
        points = []
        x = x_min
        while x <= x_max:
            y = x**2
            points.append([x, y])
            x += step
        points.append([x_max, x_max**2])
        return points

    def sigmoid_gen(self, x_min, x_max, step):
        """
        Generates points along a sigmoid curve.

        Args:
            x_min (float): Starting x value.
            x_max (float): Ending x value.
            step (float): Step size for x.

        Returns:
            list: List of [x, y] points.
        """
        points = []
        x = x_min
        while x <= x_max:
            y = 2 / (1 + math.exp(-2 * x)) - 1
            points.append([x, y])
            x += step
        points.append([x_max, 2 / (1 + math.exp(-2 * x_max)) - 1])
        return points
