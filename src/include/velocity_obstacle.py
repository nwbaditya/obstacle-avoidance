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

    def getCollisionCone(self, robot_pos, robot_vel, obstacle_pos, obstacle_vel, safety_dist):
        self.getRelativePosition(robot_pos, obstacle_pos)
        self.getRelativeVelocity(robot_vel, obstacle_vel)

        ttc = -np.dot(self.relative_pos, self.relative_pos) / np.dot(self.relative_pos, self.relative_vel)
        if ttc < 0:
            return None
        
        print(ttc)
        angle = np.arctan2(self.relative_vel[1], self.relative_vel[1])
        half_angle = np.arcsin(safety_dist / np.linalg.norm(self.relative_pos))
        left_angle = angle - half_angle
        right_angle = angle + half_angle

        return left_angle, right_angle
    
    def getVelocityObstacle(self, collision_cone):
        velocity_obstacle = []

        left_angle, right_angle = collision_cone
        angles = np.arange(left_angle, right_angle, 0.1)
        
        for angle in angles:
            for speed in np.arange(0, 20, 1):
                velocity = [speed*math.cos(angle), speed*math.sin(angle)]
                velocity_obstacle.append(velocity)

        return velocity_obstacle

    def getReachableVelocity(self, max_speed):
        reachable_velocities =[]
        for angle in np.arange(-np.pi, np.pi, 0.1):
            for speed in np.arange(0, max_speed, 1):
                velocity = [round(speed*math.cos(angle)), round(speed*math.sin(angle))]
                reachable_velocities.append(velocity)
                
        return reachable_velocities