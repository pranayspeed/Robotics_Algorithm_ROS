#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped

from tf.transformations import *

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

DRIVE_SPEED = 2.0 #m/s
DRIVE_STEER_ANGLE=0

import random

class Evader:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        
        self.turn=False

        self.evader_speed = rospy.get_param("~evader_speed")
        self.evader_steer_ang = rospy.get_param("~evader_steer_ang")
        self.evader_collision_threshold = rospy.get_param("~evader_collision_threshold")
        
        scan_topic = rospy.get_param("~scan_topic")
        pose_topic = rospy.get_param("~pose_topic")
        evader_drive_topic = rospy.get_param("~evader_drive_topic")
        car_transform_topic = rospy.get_param("~car_transform_topic")

        gt_pose_topic = rospy.get_param("~ground_truth_pose_topic")

        self.collision_detected=False
        self.old_steer_angle=0

        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.evade_walk)
        self.drive_pub = rospy.Publisher(evader_drive_topic, AckermannDriveStamped,queue_size=10)
  
        

    def evade_walk(self, msg):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "Laser"
        
        drive_msg.drive.speed = 2.0# self.evader_speed

        
        old_collision_state = self.collision_detected
        if self.laser_check_collision(msg):
            #Get Safe Angle for driving
            self.collision_detected = True
            if not old_collision_state :
                self.old_steer_angle = self.findSafeTurnAngle(msg) 

            drive_msg.drive.speed = 2.0 # self.evader_speed
        else:
            self.old_steer_angle=0
            self.collision_detected=False
            #self.evader_steer_ang
        drive_msg.drive.steering_angle = self.old_steer_angle
        self.drive_pub.publish(drive_msg)


    def laser_check_collision(self, msg):

        #TODO: add collision logic, i.e. Time to Collision concept
        #Process only +/- 10 degree data
        degree_range = 15
        resolution_per_degree = int (len(msg.ranges)/360)
        start_degree_range = 180-degree_range
        start_range_index = start_degree_range*resolution_per_degree
        end_range_index = (start_degree_range+(degree_range*2))*resolution_per_degree
        

        return self.evader_collision_threshold > min(msg.ranges[start_range_index: end_range_index])

    def findSafeTurnAngle(self, msg):
        #Not using any random choice but using a heuristic approach to find the safety angle
        #return round(random.uniform(45, 55), 2) * random.choice([1,-1])

        #Calculate the angle in degree to safely turn
        #Process only +/- 10 degree data
        degree_range = 45
        resolution_per_degree = int (len(msg.ranges)/360)
        start_degree_range = 180-degree_range
        start_range_index = start_degree_range*resolution_per_degree
        end_range_index = (start_degree_range+(degree_range*2))*resolution_per_degree
        
        
        max_value = max(msg.ranges[start_range_index: end_range_index])
        max_index = msg.ranges[start_range_index: end_range_index].index(max_value)
        safe_angle = round(max_index/resolution_per_degree,1)
        safe_angle =  safe_angle -degree_range

        return safe_angle



def main(args):
  
  rospy.init_node('evader', anonymous=True)
  evade_obj = Evader()
  rospy.spin()



if __name__=='__main__':
	main(sys.argv)