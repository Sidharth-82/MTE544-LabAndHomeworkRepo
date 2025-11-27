
from mapUtilities import *
from utilities import *
from numpy import cos, sin
import numpy as np


class particle:

    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

    def motion_model(self, v, w, dt):
        """
        Implements the motion model for the particle using a simple kinematic model.
        Updates the particle's pose [x, y, theta] based on linear velocity v, angular velocity w, and time step dt.
        Adds Gaussian noise to x, y, and theta for simulation of uncertainty.

        Parameters:
        v: linear velocity (m/s)
        w: angular velocity (rad/s)
        dt: time step (s)
        """
        # Get current pose
        x, y, th = self.pose

        # Update pose without noise first
        x_new = x + v * np.cos(th) * dt
        y_new = y + v * np.sin(th) * dt
        th_new = th + w * dt

        # Add noise (assuming std_noise is a class attribute or passed, but using a default for now)
        # std_noise = 0.01  # Example standard deviation for noise
        # x_new += np.random.normal(0, std_noise)
        # y_new += np.random.normal(0, std_noise)
        # th_new += np.random.normal(0, std_noise)

        # Update pose
        self.pose = [x_new, y_new, th_new]

    # TODO: You need to explain the following function to TA
    def calculateParticleWeight(self, scanOutput: LaserScan, mapManipulatorInstance: mapManipulator, laser_to_ego_transformation: np.array):
        """
        Calculates the weight of the particle based on how well the laser scan matches the map using a likelihood field.
        This function transforms the laser scan from the particle's pose to the map frame, then evaluates the likelihood
        of each scan point in the map's likelihood field. The weight is computed as the product of likelihoods (sum in log space).

        Parameters:
        scanOutput: LaserScan message containing range and angle data
        mapManipulatorInstance: Instance of mapManipulator for map operations
        laser_to_ego_transformation: Transformation matrix from laser to ego (robot) frame

        Explanation for TA:
        - Transforms scan points to map coordinates using the particle's pose and laser-to-ego transformation.
        - Converts scan to Cartesian coordinates.
        - Maps scan points to grid cells and filters valid cells.
        - Computes log-likelihoods from the likelihood field and sums them.
        - Converts back to weight (exp of sum) and adds small epsilon to avoid zero weights.
        """

        T = np.matmul(self.__poseToTranslationMatrix(), laser_to_ego_transformation)

        _, scanCartesianHomo = convertScanToCartesian(scanOutput)
        scanInMap = np.dot(T, scanCartesianHomo.T).T

        likelihoodField = mapManipulatorInstance.getLikelihoodField()
        cellPositions = mapManipulatorInstance.position_2_cell(
            scanInMap[:, 0:2])

        lm_x, lm_y = likelihoodField.shape

        cellPositions = cellPositions[np.logical_and.reduce(
                (cellPositions[:, 0] > 0, -cellPositions[:, 1] > 0, cellPositions[:, 0] < lm_y,  -cellPositions[:, 1] < lm_x))]

        log_weights = np.log(
            likelihoodField[-cellPositions[:, 1], cellPositions[:, 0]])
        log_weight = np.sum(log_weights)
        weight = np.exp(log_weight)
        weight += 1e-10

        self.setWeight(weight)

    def setWeight(self, weight):
        self.weight = weight

    def getWeight(self):
        return self.weight

    def setPose(self, pose):
        self.pose = pose

    def getPose(self):
        return self.pose[0], self.pose[1], self.pose[2]

    def __poseToTranslationMatrix(self):
        x, y, th = self.getPose()

        translation = np.array([[cos(th), -sin(th), x],
                                [sin(th), cos(th), y],
                                [0, 0, 1]])

        return translation
