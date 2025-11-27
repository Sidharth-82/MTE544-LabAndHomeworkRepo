"""
pid.py

This module implements PID (Proportional-Integral-Derivative) controllers
for error correction in control systems, with logging capabilities.
"""

from rclpy.time import Time
from utilities import Logger

# Controller type
P = 0  # proportional
PD = 1  # proportional and derivative
PI = 2  # proportional and integral
PID = 3  # proportional, integral, derivative


class PID_ctrl:
    """
    PID controller class for computing control outputs based on error.

    Supports P, PD, PI, and PID control modes with error history tracking
    and logging to CSV files.
    """

    def __init__(self, type_, kp=1.2, kv=0.8, ki=0.2, history_length=3, filename_="errors.csv"):
        """
        Initializes the PID controller.

        Args:
            type_ (int): Control type (P, PD, PI, PID).
            kp (float): Proportional gain.
            kv (float): Derivative gain.
            ki (float): Integral gain.
            history_length (int): Number of past errors to keep.
            filename_ (str): Filename for logging errors.
        """
        # Data for the controller
        self.history_length = history_length
        self.history = []
        self.type = type_

        # Controller gains
        self.kp = kp  # proportional gain
        self.kv = kv  # derivative gain
        self.ki = ki  # integral gain

        self.logger = Logger(filename_, headers=["error", "error_dot", "error_int", "stamp"])
        # Remember that you are writing to the file named filename_ or errors.csv the following:
        # error, error_dot, error_int and time stamp

    def update(self, stamped_error, status):
        """
        Updates the controller and returns control output.

        Args:
            stamped_error (list): [error, timestamp].
            status (bool): If False, update history but return 0.

        Returns:
            float: Control output or 0 if status is False.
        """
        if status == False:
            self.__update(stamped_error)
            return 0.0
        else:
            return self.__update(stamped_error)

    def __update(self, stamped_error):
        """
        Internal update method to compute PID terms.

        Args:
            stamped_error (list): [error, timestamp].

        Returns:
            float: Computed control output.
        """
        latest_error = stamped_error[0]
        stamp = stamped_error[1]  # This is the stamp of the msg obtained from localization odom callback

        # Add current error to history
        self.history.append(stamped_error)

        # Maintain history length
        if len(self.history) > self.history_length:
            self.history.pop(0)

        # If insufficient data points, use only the proportional gain
        if len(self.history) != self.history_length:
            return self.kp * latest_error

        # Compute the error derivative
        dt_avg = 0
        error_dot = 0

        for i in range(1, len(self.history)):
            t0 = Time.from_msg(self.history[i-1][1])
            t1 = Time.from_msg(self.history[i][1])

            dt = (t1.nanoseconds - t0.nanoseconds) / 1e9

            dt_avg += dt

            # use constant dt if the messages arrived inconsistent
            # for example dt=0.1 overwriting the calculation

            # TODO Part 5: calculate the error dot
            error_dot += (self.history[i][0] - self.history[i-1][0]) / dt

        error_dot /= (len(self.history) - 1)
        dt_avg /= (len(self.history) - 1)  # IMPORTANT: ASK TA IF SUBTRACTING 1 IS CORRECT CAUSE INTERVALS START AT INDEX 1

        # Compute the error integral
        sum_ = 0
        for hist in self.history:
            sum_ += hist[0]

        error_int = sum_ * dt_avg

        # TODO Part 4: Log your errors
        timestamp = Time.from_msg(stamp).nanoseconds / 1e9
        values_list = [latest_error, error_dot, error_int, timestamp]
        self.logger.log_values(values_list)

        # TODO Part 4: Implement the control law of P-controller
        if self.type == P:
            return latest_error * self.kp

        # TODO Part 5: Implement the control law corresponding to each type of controller
        elif self.type == PD:
            return latest_error * self.kp + error_dot * self.kv

        elif self.type == PI:
            return latest_error * self.kp + error_int * self.ki

        elif self.type == PID:
            return latest_error * self.kp + error_dot * self.kv + error_int * self.ki
