import math
import numpy as np

class VelocityObstacle:
    def __init__(self) -> None:
        self.theta = 0
        self.theta_bound_right = 0
        self.theta_bound_left = 0
        self.bound_left = [0,0]
        self.bound_right = [0,0]
        self.euclidean_distance = 0
        self.translational_vel = [0,0]
        self.relative_pos = [0,0]
        self.relative_vel = [0,0]

    def getRelativePosition(self, robot_pos, obstacle_pos):
        self.relative_pos = [obstacle_pos[0]-robot_pos[0], obstacle_pos[1]-robot_pos[1]]

    def getRelativeVelocity(self, robot_vel, obstacle_vel):
        self.relative_vel = [obstacle_vel[0]-robot_vel[0], obstacle_vel[1]-robot_vel[1]]

    def getEuclideanDistance(self, robot_pos, obstacle_pos):
        self.getRelativePosition(robot_pos, obstacle_pos)
        self.euclidean_distance = math.sqrt((self.relative_pos[0]**2) + (self.relative_pos[1]**2))

    def getTheta(self, robot_pos, obstacle_pos):
        self.getRelativePosition(robot_pos, obstacle_pos)
        self.theta = math.atan2(self.relative_pos[1], self.relative_pos[0])

    def getThetaBoundLeft(self, robot_pos, obstacle_pos, robot_radius, obstacle_radius):
        self.getEuclideanDistance(robot_pos, obstacle_pos)
        self.getTheta(robot_pos, obstacle_pos)
        self.theta_bound_left = self.theta + ((robot_radius + obstacle_radius) / self.euclidean_distance)
    
    def getBoundLeft(self, robot_pos, obstacle_pos, robot_radius, obstacle_radius):
        self.getThetaBoundLeft(robot_pos, obstacle_pos, robot_radius, obstacle_radius)
        self.bound_left[0] = math.cos(self.theta_bound_left)
        self.bound_left[1] = math.sin(self.theta_bound_left)

    def getThetaBoundRight(self, robot_pos, obstacle_pos, robot_radius, obstacle_radius):
        self.getEuclideanDistance(robot_pos, obstacle_pos)
        self.getTheta(robot_pos, obstacle_pos)
        self.theta_bound_right = self.theta - ((robot_radius + obstacle_radius) / self.euclidean_distance)

    def getBoundRight(self, robot_pos, obstacle_pos, robot_radius, obstacle_radius):
        self.getThetaBoundRight(robot_pos, obstacle_pos, robot_radius, obstacle_radius)
        self.bound_right[0] = math.cos(self.theta_bound_right)
        self.bound_right[1] = math.sin(self.theta_bound_right)

    def getTranslationalVelocity(self, robot_pos, obstacle_vel):
        self.translational_vel[0] = robot_pos[0] + obstacle_vel[0]
        self.translational_vel[1] = robot_pos[1] + obstacle_vel[1]

