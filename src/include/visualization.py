import cv2
import numpy as np

class Visualization():
    def __init__(self):
        self.img_size = (800,800)
        self.img = np.zeros((self.img_size[1], self.img_size[0], 3), dtype=np.uint8)
        self.bg_color = (255, 255, 255)
        self.img[:] = self.bg_color
        self.robot_pos= (int(self.img_size[1]/2), int(self.img_size[0]/2))
        self.max_obs_distance = 2

    def showImage(self):
        cv2.waitKey(1)

    def showVelocityObstacleFrame(self, objects):
        self.img[:] = self.bg_color
        for object in objects:
            pX = self.robot_pos[0] - int(object.position[0]/self.max_obs_distance *self.img_size[1]/2)
            pY = self.robot_pos[1] - int(object.position[1]/self.max_obs_distance *self.img_size[0]/2)

            print("OBS COORDINATE VIS")
            print(pX, pY)

            cv2.circle(self.img, (pY, pX), 10, (255,255,0), -1)

        cv2.circle(self.img, (self.robot_pos), 10, (0,255,0), -1)
        cv2.imshow('Velocity Obstacle', self.img)
        cv2.waitKey(1)