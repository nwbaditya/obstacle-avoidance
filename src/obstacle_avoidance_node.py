#!/usr/bin/env python3

import rospy
import zed_interfaces.msg._ObjectsStamped
import zed_interfaces.msg._Object
import geometry_msgs.msg._PoseStamped

import math
import numpy as np
import threading

from include.robot import Robot
from include.obstacle import Obstacle
from include.velocity_obstacle import VelocityObstacle
from include.visualization import Visualization

import cv2

ROBOT_RADIUS = 0.5
OBSTACLE_RADIUS = 0.5


vis = Visualization()
obstacles_vis = []

class ObstacleAvoidance:
    def __init__(self):
        self.robot = Robot(ROBOT_RADIUS)

        rospy.Subscriber("/zed2i/zed_node/obj_det/objects", zed_interfaces.msg.ObjectsStamped, self.zed_od_callback)
        rospy.Subscriber("/zed2i/zed_node/pose", geometry_msgs.msg.PoseStamped, self.robot_pose_callback)
    
    def zed_od_callback(self, object):

        rvo_all = []

        if(len(object.objects) == 0):
            print("No Object Detected")
            return
        
        for i in range(len(object.objects)):
            obstacle = Obstacle(object.objects[i].label_id)
            velocity_obstacle = VelocityObstacle()
            global obstacles_vis

            obstacle.position[0] = object.objects[i].position[0]
            obstacle.position[1] = object.objects[i].position[1]

            obstacles_vis.append(obstacle)

            obstacle.velocity[0] = object.objects[i].velocity[0]
            obstacle.velocity[1] = object.objects[i].velocity[1]

            velocity_obstacle.getBoundLeft(self.robot.position, obstacle.position, ROBOT_RADIUS, OBSTACLE_RADIUS)
            velocity_obstacle.getBoundRight(self.robot.position, obstacle.position, ROBOT_RADIUS, OBSTACLE_RADIUS)
            velocity_obstacle.getTranslationalVelocity(self.robot.position, obstacle.velocity)


            radius = ROBOT_RADIUS + OBSTACLE_RADIUS
            rvo = [velocity_obstacle.translational_vel, velocity_obstacle.bound_left,   
                   velocity_obstacle.bound_right, velocity_obstacle.euclidean_distance, radius]
            
            rvo_all.append(rvo)

        v_des = [1,1]

        va_post = self.intersect(self.robot.position, v_des, rvo_all)
        v_opt = va_post
        print(v_opt)

    def robot_pose_callback(self, robot):
        self.robot.position[0] = robot.pose.position.x
        self.robot.position[1] = robot.pose.position.y


    def intersect(self, robot_pos, robot_vel, rvo_all):
        norm_v = self.distance(robot_vel, [0, 0])
        suitable_v = []
        unsuitable_v = []
        for theta in np.arange(0, 2*math.pi, 0.1):
            for rad in np.arange(0.02, norm_v+0.02, norm_v/5.0):
                new_v = [rad*math.cos(theta), rad*math.sin(theta)]
                suit = True
                for rvo in rvo_all:
                    p_0 = rvo[0]
                    left = rvo[1]
                    right = rvo[2]
                    dif = [new_v[0]+robot_pos[0]-p_0[0], new_v[1]+robot_pos[1]-p_0[1]]
                    theta_dif = math.atan2(dif[1], dif[0])
                    theta_right = math.atan2(right[1], right[0])
                    theta_left = math.atan2(left[1], left[0])
                    if self.inBetween(theta_right, theta_dif, theta_left):
                        suit = False
                        break
                if suit:
                    suitable_v.append(new_v)
                else:
                    unsuitable_v.append(new_v)   


        new_v = robot_vel[:]
        suit = True
        for rvo in rvo_all:
            p_0 = rvo[0]
            left = rvo[1]
            right = rvo[2]
            dif = [new_v[0] + robot_pos[0] - p_0[0], new_v[1] + robot_pos[1] - p_0[1]]
            theta_dif = math.atan2(dif[1], dif[0])
            theta_right = math.atan2(right[1], right[0])
            theta_left = math.atan2(left[1], left[0])

            if self.inBetween(theta_right, theta_dif, theta_left):
                suit = False
                break
        
        if suit:
            suitable_v.append(new_v)
        else:
            unsuitable_v.append(new_v)

        if suitable_v:
            print('suitable found')
            # print(suitable_v)
            va_post = min(suitable_v, key= lambda v: self.distance(v, robot_vel))
            print(va_post)
            new_v = va_post[:]
            for rvo in rvo_all:
                p_0 = rvo[0]
                left = rvo[1]
                right = rvo[2]
                dif = [new_v[0]+robot_pos[0]-p_0[0], new_v[1]+robot_pos[1]-p_0[1]]
                theta_dif = math.atan2(dif[1], dif[0])
                theta_right = math.atan2(right[1], right[0])
                theta_left = math.atan2(left[1], left[0])

        else:
            print('suitable not found')
            tc_V = dict()
            for unsuit_v in unsuitable_v:
                tc_V[tuple(unsuit_v)] = 0
                tc = []
                for rvo in rvo_all:
                    p_0 = rvo[0]
                    left = rvo[1]
                    right = rvo[2]
                    dist = rvo[3]
                    radius = rvo[4]
                    dif = [unsuit_v[0] + robot_pos[0] - p_0[0], unsuit_v[1] + robot_pos[1] - p_0[1]]
                    theta_dif = math.atan2(dif[1], dif[0])
                    theta_right = math.atan2(right[1], right[0])
                    theta_left = math.atan2(left[1], left[0])

                    if self.inBetween(theta_right, theta_dif, theta_left):
                        small_theta = abs(theta_dif- 0.5 * (theta_left+theta_right))
                        if abs(dist*math.sin(small_theta)) >= radius:
                            radius = abs(dist*math.sin(small_theta))
                        
                        big_theta = math.asin(abs(dist*math.sin(small_theta))/radius)
                        dist_tg = abs(dist*math.cos(small_theta)) - abs(radius*math.cos(big_theta))

                        if dist_tg < 0:
                            dist_tg = 0
                        tc_v = dist_tg/self.distance(dif, [0.0])
                        tc.append(tc_v)
                    
                tc_V[tuple(unsuit_v)] = min(tc) + 0.001
            WT = 0.2
            va_post = min(unsuitable_v, key=lambda v: ((WT/tc_V[tuple(v)]) + self.distance(v, robot_vel)))
        return va_post

    def inBetween(self, theta_right, theta_dif, theta_left):
        if abs(theta_right - theta_left) <= math.pi:
            # print(theta_left, theta_dif, theta_right)
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        else:
            if (theta_left <0) and (theta_right >0):
                theta_left += 2*math.pi
                if theta_dif < 0:
                    theta_dif += 2*math.pi
                if theta_right <= theta_dif <= theta_left:
                    return True
                else:
                    return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*math.pi
            if theta_dif < 0:
                theta_dif += 2*math.pi
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False
        
    def distance(self, pose1, pose2):
        return math.sqrt((pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2)

def visualization_thread():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        global obstacles_vis
        vis.showVelocityObstacleFrame(obstacles_vis)
        obstacles_vis = []
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node_py')
    node = ObstacleAvoidance()

    worker = threading.Thread(target=visualization_thread)
    worker.start()
    rospy.spin()