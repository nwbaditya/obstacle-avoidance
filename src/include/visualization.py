import cv2
import numpy as np
import math


class Visualization():
    def __init__(self, robot_radius, obstacle_radius):
        self.img_size = (800,800)
        self.img_vo = np.zeros((self.img_size[1], self.img_size[0], 3), dtype=np.uint8)
        self.img_cc = np.zeros((self.img_size[1], self.img_size[0], 3), dtype=np.uint8)
        self.bg_color = (255, 255, 255)
        self.img_vo[:] = self.bg_color
        self.img_cc[:] = self.bg_color
        self.max_obs_distance = 30
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.robot_radius_px = self.robot_radius/self.max_obs_distance/2*self.img_size[0]
        self.obstacle_radius_px = int(self.obstacle_radius/self.max_obs_distance/2*self.img_size[0])
        self.robot_pos= (int(self.img_size[1]/2), int(self.img_size[0]/2))
        self.obstacle_vel =[0, 0]
        self.origin = [int(self.img_size[0]/2), int(self.img_size[1]/2)]

    def showImage(self):
        cv2.waitKey(1)

    def showVelocityObstacleFrame(self, suitable_v, unsuitable_v, robot_vel_des, robot_vel_opt, obstacles):
        self.img_vo[:] =self.bg_color
        # for obstacle in obstacles:
        #     obs_pX = self.robot_pos[0] - int(obstacle[0].position[0]/self.max_obs_distance *self.img_size[1]/2)
        #     obs_pY = self.robot_pos[1] - int(obstacle[0].position[1]/self.max_obs_distance *self.img_size[0]/2)
        #     obs_coordinate_vis = (obs_pY,obs_pX)
        #     robot_coordinate_vis = (self.robot_pos[1], self.robot_pos[0])
        #     dist_text_coordinate_vis = (obs_pY+10, obs_pX+10)
        #     theta_text_coordinate_vis = (obs_pY+10, obs_pX-10)

        #     euclidean_distance = np.linalg.norm((obstacle[0].position[0], obstacle[0].position[1]))
        #     euclidean_distance_text = "Distance[cm]: " + str(int(euclidean_distance))
        #     theta_text = "Theta[rad]: "+ str(obstacle[1].theta)

        #     cv2.putText(self.img_vo, euclidean_distance_text, org= dist_text_coordinate_vis, color= (255,0,0), fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL, fontScale= 1)
        #     cv2.putText(self.img_vo, theta_text, org= theta_text_coordinate_vis, color= (255,0,0), fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL, fontScale= 1)

        #     cv2.line(self.img_vo, robot_coordinate_vis, obs_coordinate_vis, color=(255,255,0))
        #     cv2.circle(self.img_vo, (obs_pY, obs_pX), 10, (255,255,0), -1)

        if suitable_v:
            for v in suitable_v:
                vx = self.origin[0] - int(v[0]/self.max_obs_distance*self.img_size[0]/2)
                vy = self.origin[1] - int(v[1]/self.max_obs_distance*self.img_size[1]/2)

                cv2.circle(self.img_vo, (int(vy), int(vx)), 2, (0,255,0), -1)

        if unsuitable_v:
            for v in unsuitable_v:
                vx = self.origin[0] - int(v[0]/self.max_obs_distance*self.img_size[0]/2)
                vy = self.origin[1] - int(v[1]/self.max_obs_distance*self.img_size[1]/2)

                cv2.circle(self.img_vo, (int(vy), int(vx)), 2, (0,0,255), -1)

        vx_opt = self.robot_pos[0] - int(robot_vel_opt[0]/self.max_obs_distance*self.img_size[0]/2)
        vy_opt = self.robot_pos[1] - int(robot_vel_opt[1]/self.max_obs_distance*self.img_size[1]/2)
        robot_vel_opt_px = [vy_opt, vx_opt]

        vx_des = self.origin[0] - int(robot_vel_des[0]/self.max_obs_distance*self.img_size[0]/2)
        vy_des = self.origin[1] - int(robot_vel_des[1]/self.max_obs_distance*self.img_size[1]/2)
        robot_vel_des_px = [vy_des, vx_des]

        cv2.arrowedLine(self.img_vo, self.origin, robot_vel_des_px, (0,0 ,100), 2)
        cv2.arrowedLine(self.img_vo, self.origin, robot_vel_opt_px, (255, 0, 0), 2)

        # cv2.circle(self.img, self.robot_pos, int(self.robot_radius_px), (0,255,0), -1)
        # cv2.circle(self.img, self.robot_pos, int(self.robot_radius_px + self.obstacle_radius_px), (0,0,255), 2)
        cv2.imshow('Velocity Obstacle', self.img_vo)
        cv2.waitKey(1)
            

    def showCollisionConeFrame(self, obstacles):
        self.img[:] = self.bg_color
        for obstacle in obstacles:
            pX = self.robot_pos[0] - int(obstacle[0].position[0]/self.max_obs_distance *self.img_size[1]/2)
            pY = self.robot_pos[1] - int(obstacle[0].position[1]/self.max_obs_distance *self.img_size[0]/2)

            euclidean_distance = np.linalg.norm((obstacle[0].position[0], obstacle[0].position[1]))

            euclidean_distance_px = euclidean_distance/self.max_obs_distance *self.img_size[1]/2

            obj_coordinate_vis = (pY,pX)
            robot_coordinate_vis = (self.robot_pos[1], self.robot_pos[0])
            obstacle_vel_px = (int(obstacle[1].translational_vel[1]/self.max_obs_distance*self.img_size[0]), int(obstacle[1].translational_vel[0]/self.max_obs_distance*self.img_size[1]))

            euclidean_distance_text = "Distance[cm]: " + str(int(euclidean_distance))
            theta_text = "Theta[rad]: "+ str(obstacle[1].theta)
            
            # self.drawPolyUsingAngle((robot_coordinate_vis[0] + obstacle_vel_px[0], robot_coordinate_vis[1]+obstacle_vel_px[1]), 
            #                         euclidean_distance_px, obstacle[1].theta_bound_left, obstacle[1].theta_bound_right)

            self.drawPolyUsingAngle((robot_coordinate_vis[0], robot_coordinate_vis[1]), 
                                    euclidean_distance_px, obstacle[1].theta_bound_left, obstacle[1].theta_bound_right)

        for obstacle in obstacles:
            pX = self.robot_pos[0] - int(obstacle[0].position[0]/self.max_obs_distance *self.img_size[1]/2)
            pY = self.robot_pos[1] - int(obstacle[0].position[1]/self.max_obs_distance *self.img_size[0]/2)

            euclidean_distance = np.linalg.norm((obstacle[0].position[0], obstacle[0].position[1]))
            euclidean_distance_px = euclidean_distance/self.max_obs_distance *self.img_size[1]/2

            obj_coordinate_vis = (pY,pX)
            robot_coordinate_vis = (self.robot_pos[1], self.robot_pos[0])
            dist_text_coordinate_vis = (pY+10, pX+10)
            theta_text_coordinate_vis = (pY+10, pX-10)

            euclidean_distance_text = "Distance[cm]: " + str(int(euclidean_distance))
            theta_text = "Theta[rad]: "+ str(obstacle[1].theta)
            
            cv2.putText(self.img, euclidean_distance_text, org= dist_text_coordinate_vis, color= (255,0,0), fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL, fontScale= 1)
            cv2.putText(self.img, theta_text, org= theta_text_coordinate_vis, color= (255,0,0), fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL, fontScale= 1)

            cv2.line(self.img, robot_coordinate_vis, obj_coordinate_vis, color=(255,255,0))
            cv2.circle(self.img, (pY, pX), 10, (255,255,0), -1)

        cv2.circle(self.img, self.robot_pos, int(self.robot_radius_px), (0,255,0), -1)
        cv2.circle(self.img, self.robot_pos, int(self.robot_radius_px + self.obstacle_radius_px), (0,0,255), 2)
        cv2.imshow('Collision Cone', self.img)
        cv2.waitKey(1)

    def drawLineUsingAngle(self, origin, length, angle):
        x2 = origin[0] - length* math.cos(angle)
        y2 = origin[1] - length* math.sin(angle)

        end_point = (int(y2),int(x2))
        cv2.line(self.img, origin, end_point,color=(255,0,0))

    def drawPolyUsingAngle(self, origin, length, angle_left, angle_right):
        x2 = origin[0] - length* math.cos(angle_left)
        y2 = origin[1] - length* math.sin(angle_left)

        x3 = origin[0] - length* math.cos(angle_right)
        y3 = origin[1] - length* math.sin(angle_right)

        end_point1 = (int(y2),int(x2))
        end_point2 = (int(y3),int(x3))
        pts = np.array([list(origin),list(end_point1),list(end_point2)])

        cv2.fillPoly(self.img, pts=[pts],color=(220,220,220))