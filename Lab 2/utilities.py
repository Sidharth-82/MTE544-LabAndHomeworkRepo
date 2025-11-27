"""
utilities.py

This module provides utility classes and functions for logging data,
reading CSV files, quaternion to Euler conversions, and error calculations
used in robot navigation and control.
"""

from math import atan2, asin, sqrt

M_PI = 3.1415926535


class Logger:
    """
    Utility class for logging data to CSV files.

    Handles writing headers and appending data rows to a specified file.
    """

    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        """
        Initializes the logger with a filename and headers.

        Args:
            filename (str): Path to the CSV file.
            headers (list): List of column headers.
        """
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str = ""

            for header in headers:
                header_str += header
                header_str += ", "

            header_str += "\n"

            file.write(header_str)

    def log_values(self, values_list):
        """
        Logs a list of values to the CSV file.

        Args:
            values_list (list): Values to log as a row.
        """
        with open(self.filename, 'a') as file:
            vals_str = ""

            for value in values_list:
                vals_str += f"{value}, "

            vals_str += "\n"

            file.write(vals_str)

    def save_log(self):
        """
        Placeholder for saving logs (currently does nothing).
        """
        pass


class FileReader:
    """
    Utility class for reading CSV files.

    Parses headers and data rows from CSV files.
    """

    def __init__(self, filename):
        """
        Initializes the file reader with a filename.

        Args:
            filename (str): Path to the CSV file to read.
        """
        self.filename = filename

    def read_file(self):
        """
        Reads the CSV file and returns headers and data table.

        Returns:
            tuple: (headers list, data table as list of lists).
        """
        read_headers = False

        table = []
        headers = []
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values = line.strip().split(',')

                    for val in values:
                        if val == '':
                            break
                        headers.append(val.strip())

                    read_headers = True
                    break

            next(file)

            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')

                row = []

                for val in values:
                    if val == '':
                        break
                    row.append(float(val.strip()))

                table.append(row)

        return headers, table


# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.

    Returns only the yaw angle for 2D navigation.

    Args:
        quat: Quaternion object with x, y, z, w attributes.

    Returns:
        float: Yaw angle in radians.
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x**2 + y**2)
    roll = atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    pitch = asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y**2 + z**2)
    yaw = atan2(t3, t4)
    return yaw


# TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
    """
    Calculates the Euclidean distance (linear error) between current and goal positions.

    Args:
        current_pose (list): [x, y, theta, timestamp].
        goal_pose (list): [x, y].

    Returns:
        float: Linear error (distance).
    """
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    current_x, current_y = current_pose[0], current_pose[1]
    goal_x, goal_y = goal_pose[0], goal_pose[1]
    error_linear = sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)

    return error_linear


# TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):
    """
    Calculates the angular error between current orientation and desired orientation to goal.

    Args:
        current_pose (list): [x, y, theta, timestamp].
        goal_pose (list): [x, y].

    Returns:
        float: Angular error in radians, normalized to [-pi, pi].
    """
    # Compute the angular error
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Use atan2 to find the desired orientation
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal

    current_x, current_y, current_theta = current_pose[0], current_pose[1], current_pose[2]
    goal_x, goal_y = goal_pose[0], goal_pose[1]

    # Desired orientation to reach the goal
    desired_theta = atan2(goal_y - current_y, goal_x - current_x)

    # Angular error
    error_angular = desired_theta - current_theta

    # Normalize the angular error to [-pi, pi]
    while error_angular > M_PI:
        error_angular -= 2 * M_PI
    while error_angular < -M_PI:
        error_angular += 2 * M_PI

    return error_angular
