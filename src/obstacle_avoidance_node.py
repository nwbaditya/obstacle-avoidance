#!/usr/bin/env python3

import rospy
import zed_interfaces.msg._ObjectsStamped
import zed_interfaces.msg._Object
import geometry_msgs.msg._PoseStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from main_controller.msg import ControllerData

import math
import numpy as np
import threading

from include.robot import Robot
from include.obstacle import Obstacle
from include.velocity_obstacle import VelocityObstacle
from include.visualization import Visualization

import cv2

ROBOT_RADIUS = 30 #cm
OBSTACLE_RADIUS = 20 #cm

ROBOT_MAX_SPEED = 20
OBSTACLE_DETECTION_THRESHOLD = 300

current_v = [5,0]
desired_v = [10,-10]


vis = Visualization(ROBOT_RADIUS, OBSTACLE_RADIUS, ROBOT_MAX_SPEED)
obstacles_vis = []
suitable_v_vis = []
unsuitable_v_vis = []
robot_vel_opt_vis = [0,0]
robot_vel_des_vis = [0,0]


class ObstacleAvoidance:
    def __init__(self):
        self.robot = Robot(ROBOT_RADIUS)
        self.robot_opt_vel = [0,0]

        self.obstacle_detected_pub = rospy.Publisher("/obstacle_detected",Bool, queue_size=1)
        self.ov_velocity_pub = rospy.Publisher("/obs_vel", Twist, queue_size=1)
        self.crashed_pub = rospy.Publisher("/crashed", Bool, queue_size=1)

        rospy.Subscriber("/zed2i/zed_node/obj_det/objects", zed_interfaces.msg.ObjectsStamped, self.zed_od_callback)
        rospy.Subscriber("/zed2i/zed_node/pose", geometry_msgs.msg.PoseStamped, self.robot_pose_callback)
        rospy.Subscriber("/pure_pursuit_vel", geometry_msgs.msg.Twist, self.pure_pursuit_callback)
        rospy.Subscriber("/robot/cmd_vel", ControllerData, self.cmd_vel_callback)

    def zed_od_callback(self, object):
        global current_v
        global suitable_v_vis
        global unsuitable_v_vis
        global robot_vel_opt_vis
        global robot_vel_des_vis
        
        vo_all = []

        print("OBSTACLE AVOIDANCE NODE IS RUNNING")
        if(len(object.objects) == 0):
            global suitable_v_vis
            print("No Object Detected")
            msg_obj = Bool()
            msg_obj.data = False
            self.obstacle_detected_pub.publish(msg_obj)
            for theta in np.arange(-np.pi, np.pi, 0.1):
                for speed in np.arange(0, ROBOT_MAX_SPEED, 1):
                    new_v = [round(speed*np.cos(theta)), round(speed*np.sin(theta))]
                    suitable_v_vis.append(new_v)
        else:
            # self.robot.velocity = current_v
            # self.robot.desired_velocity = desired_v
            for i in range(len(object.objects)):
                obstacle = Obstacle(object.objects[i].label_id)
                velocity_obstacle = VelocityObstacle()
                global obstacles_vis

                #Convert to Cm
                obstacle.position[0] = object.objects[i].position[0]*100
                obstacle.position[1] = object.objects[i].position[1]*100

                obstacle.velocity[0] = object.objects[i].velocity[0]*100
                obstacle.velocity[1] = object.objects[i].velocity[1]*100

                velocity_obstacle.getBoundLeft(self.robot.position, obstacle.position, ROBOT_RADIUS, OBSTACLE_RADIUS)
                velocity_obstacle.getBoundRight(self.robot.position, obstacle.position, ROBOT_RADIUS, OBSTACLE_RADIUS)
                velocity_obstacle.getRelativeVelocity(self.robot.velocity, [0,0])
                # velocity_obstacle.getTranslationalVelocity(self.robot.position, obstacle.velocity)
                velocity_obstacle.getTranslationalVelocity(self.robot.position, velocity_obstacle.relative_vel)

                if(velocity_obstacle.euclidean_distance < ROBOT_RADIUS + OBSTACLE_RADIUS):
                    print("ROBOT CRASHED")
                    msg_crashed = Bool()
                    msg_crashed.data = True
                    self.crashed_pub.publish(msg_crashed)
                else:
                    msg_crashed = Bool()
                    msg_crashed.data = False
                    self.crashed_pub.publish(msg_crashed)

                obs = [obstacle, velocity_obstacle]
                obstacles_vis.append(obs)
                safe_radius = ROBOT_RADIUS + OBSTACLE_RADIUS + 10
                vo = [velocity_obstacle.translational_vel,
                      velocity_obstacle.bound_left,
                      velocity_obstacle.bound_right,
                      velocity_obstacle.euclidean_distance,
                      safe_radius
                      ]
                
                vo_all.append(vo)

                if(velocity_obstacle.euclidean_distance < OBSTACLE_DETECTION_THRESHOLD):
                    msg_obj = Bool()
                    msg_obj.data = True
                    self.obstacle_detected_pub.publish(msg_obj)
                else:
                    msg_obj = Bool()
                    msg_obj.data = False
                    self.obstacle_detected_pub.publish(msg_obj)
                    # return
            
            self.robot_opt_vel = self.intersect(self.robot.position, self.robot.desired_velocity, vo_all)
            robot_vel_opt_vis = self.robot_opt_vel
            print(robot_vel_des_vis)
            #Publish
            msg_vel = Twist()
            msg_vel.linear.x = self.robot_opt_vel[0]
            msg_vel.linear.y = self.robot_opt_vel[1]
            msg_vel.linear.z = 0
            msg_vel.angular.z = 0
            self.ov_velocity_pub.publish(msg_vel)

        robot_vel_des_vis = self.robot.desired_velocity


    def intersect(self, robot_pose, robot_desired_vel, vo_all):
        suitable_v = []
        unsuitable_v = []

        global suitable_v_vis
        global unsuitable_v_vis

        for theta in np.arange(-np.pi, np.pi, 0.05):
            for speed in np.arange(0, ROBOT_MAX_SPEED, 1):
                new_v = [round(speed*np.cos(theta)), round(speed*np.sin(theta))]
                suit = True
                for vo in vo_all:
                    p0 = vo[0]
                    bound_left = vo[1]
                    bound_right = vo[2]
                    dif = [new_v[0] + robot_pose[0] - p0[0], new_v[1] + robot_pose[1] - p0[1]]
                    theta_dif = np.arctan2(dif[1], dif[0])
                    theta_left = np.arctan2(bound_left[1], bound_left[0])
                    theta_right = np.arctan2(bound_right[1], bound_right[0])

                    if self.inBetween(theta_left, theta_dif, theta_right):
                        suit = False

                    if suit:
                        suitable_v.append(new_v)
                    else:
                        unsuitable_v.append(new_v)
        

        if suitable_v:
            robot_vel_post = min(suitable_v, key= lambda v: self.distance(v, robot_desired_vel))
        
        else:
            print('unsuitable_v')
            tc_v = dict()
            for v in unsuitable_v:
                tc_v[tuple(v)] = 0
                tc = []
                for vo in vo_all:
                    p0 = vo[0]
                    bound_left = vo[1]
                    bound_right = vo[2]
                    dist = vo[3]
                    radius = vo[4]
                    dif = [v[0] + robot_pose[0] - p0[0], v[1] + robot_pose[1] - p0[1]]
                    theta_dif = np.arctan2(dif[1], dif[0])
                    theta_left = np.arctan2(bound_left[1], bound_left[0])
                    theta_right = np.arctan2(bound_right[1], bound_right[0])

                    if self.inBetween(theta_left, theta_dif, theta_right):
                        small_theta = abs(theta_dif-0.5*(theta_left + theta_right))
                        if abs(dist*np.sin(small_theta)) >= radius:
                            radius = abs(dist*np.sin(small_theta))

                        big_theta = np.arcsin(abs(dist*np.sin(small_theta))/radius)
                        dist_tg = abs(dist*np.cos(small_theta)) - abs(radius*np.cos(big_theta))

                        if(dist_tg) < 0:
                            dist_tg = 0
                        
                        v_tc = dist_tg/np.linalg.norm(dif, [0,0])
                        tc.append(v_tc)
                tc_v[tuple(v)] = min(tc) + 0.001
                WT = 0.2
                robot_vel_post = min(unsuitable_v, key=lambda v: ((WT/tc_v[tuple(v)]) + self.distance(v, robot_desired_vel)))
        
        suitable_v_vis = suitable_v[:]
        unsuitable_v_vis = unsuitable_v[:]
        return robot_vel_post

    def inBetween(self, theta_left, theta_dif, theta_right):
        if abs(theta_right - theta_left) <= np.pi:
            if theta_left >= theta_dif >= theta_right:
                return True
            else:
                return False
        
        else:
            if(theta_left < 0 and theta_right > 0):
                theta_left+=2*np.pi
                if(theta_dif < 0):
                    theta_dif+=2*np.pi
                if(theta_left >= theta_dif >= theta_right):
                    return True
                else: 
                    return False   
            
            if(theta_left < theta_right):
                theta_right-=2*np.pi
                theta_dif-=2*np.pi

                if(theta_left >= theta_dif >= theta_right):
                    return True
                else:
                    return False
    

    def distance(self,pose1, pose2):
        """ compute Euclidean distance for 2D """
        return math.sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001

    def robot_pose_callback(self, robot):
        self.robot.position[0] = 0
        self.robot.position[1] = 0

    def pure_pursuit_callback(self, msg):
        self.robot.desired_velocity[0] = msg.linear.x
        self.robot.desired_velocity[1] = msg.linear.y
        # print(self.robot.desired_velocity)

    def cmd_vel_callback(self, msg):
        self.robot.velocity[0] = msg.data[0]
        self.robot.velocity[1] = msg.data[1]



def visualization_thread():
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        global obstacles_vis
        global suitable_v_vis
        global unsuitable_v_vis
        global robot_vel_opt_vis
        global robot_vel_des_vis

        vis.showVelocityObstacleFrame(
            suitable_v=suitable_v_vis,
            unsuitable_v= unsuitable_v_vis, 
            obstacles=obstacles_vis, 
            robot_vel_des = robot_vel_des_vis,
            robot_vel_opt= robot_vel_opt_vis)
        print(robot_vel_des_vis, robot_vel_opt_vis)

        obstacles_vis = []
        suitable_v_vis = []
        unsuitable_v_vis = []
        robot_vel_opt_vis = [0,0]
        robot_vel_des_vis = [0,0]
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node_py')
    node = ObstacleAvoidance()

    vis_worker = threading.Thread(target=visualization_thread)
    vis_worker.start()
    rospy.spin()