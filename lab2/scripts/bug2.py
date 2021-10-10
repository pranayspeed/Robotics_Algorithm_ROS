#!/usr/bin/env python
import sys

from numpy.core.defchararray import array
import rospy


from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Point, Vector3, Quaternion


from visualization_msgs.msg import Marker

import numpy as np
import random
import tf
import math

state_list = ["GOAL_SEEK","WALL_FOLLOW"]
class Bug2:
    """ Implement ransac algo to display lines
    """
    def __init__(self):
        

        drive_topic = rospy.get_param("~goal_seeker_drive_topic")
        odom_topic = rospy.get_param("~odom_topic")


        self.start_pt = rospy.get_param("~start_pt")
        self.goal_pt = rospy.get_param("~goal_pt")

        ransac_lines_topic = rospy.get_param("~ransac_lines")

        self.goal_vec = np.array([self.goal_pt[0],self.goal_pt[1]]) - np.array([self.start_pt[0],self.start_pt[1]])


        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.goal_walker)
        self.odom_sub = rospy.Subscriber(ransac_lines_topic, Marker, self.update_wall)


        self.drive_pub = rospy.Publisher(drive_topic, Twist,queue_size=10)

        self.current_state=state_list[0]

        self.current_wall=None
        self.current_direction=self.goal_vec

        self.collision_threshold = rospy.get_param("~collision_threshold")


    def update_wall(self, msg):
        
        p1 = msg.points[0]
        p2 = msg.points[1]
        p1 = np.array([p1.x,p1.y])
        p2 = np.array([p2.x,p2.y])
        d = abs(np.cross(p2-p1, p1)/np.linalg.norm(p2-p1))

        self.current_wall=None
        if d < self.collision_threshold:
            self.current_wall=[p1,p2]
            print("Wall found")


  
    def check_if_encountered_goal_line(self, pt):

        goal_unit_vec = self.goal_vec / np.linalg.norm(self.goal_vec)         # unit vector from r0 to r1
        pt_vec = np.array(pt) - np.array(self.start_pt)             # vector from r0 to pt
        dist_proj = np.dot(pt_vec, goal_unit_vec)   # projection (length) of r onto r01u
        proj_vector = goal_unit_vec * dist_proj         # projection vector
        pt_on_line = self.start_pt + proj_vector           # point on line
        
        dist = np.linalg.norm(np.array(pt) - pt_on_line)
        print("goal line distance", dist)
        return abs(dist) < 0.01

    def rotation_angle(self, line1, line2):
        signed_angle = math.atan2(line2[1],line2[0]) - math.atan2(line1[1],line1[0])
        signed_angle = math.degrees(signed_angle)
        return signed_angle
        # print("signed angle", signed_angle)
        # unit_vector_1 = line1 / np.linalg.norm(line1)
        # unit_vector_2 = line2 / np.linalg.norm(line2)
        # dot_product = np.dot(unit_vector_1, unit_vector_2)
        # return np.arccos(dot_product)

       


    def check_if_collison(self):
        if self.current_wall:
            return True
        return False
    def goal_walker(self, msg):
        
        twist_msg = Twist()
        twist_msg.linear= Vector3(0,0,0)
        twist_msg.angular= Vector3(0,0,0)

        state_x = msg.pose.pose.position.x
        state_y = msg.pose.pose.position.y
        
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        current_angle = euler[2] #Yaw

        current_direction = np.array([math.cos(current_angle),math.sin(current_angle)])

        if self.current_state == state_list[0]:
            #Goal seeking
            print("Goal Seeking")
            #check collision
            if self.check_if_collison():
                #change state to wall follow direction parallel to wall
                self.current_state = state_list[1]
                
                #get angle between obstacle and current direction and rotate
                self.current_direction = np.array(self.current_wall[0]) - np.array(self.current_wall[1]) 

                self.current_direction = self.current_direction / np.linalg.norm(self.current_direction) 
                
        else:
            # wall Following
            print("Wall following")
            #check if intersecting m-line
            if self.check_if_encountered_goal_line([state_x,state_y]):                
                #change state to goal seeking
                self.current_state = state_list[0]
                
                #get angle between m_line and current direction and rotate
                #self.current_direction = self.goal_vec
                self.current_direction = np.array([self.goal_pt[0],self.goal_pt[1]])- np.array([state_x,state_y])
                self.current_direction = self.current_direction / np.linalg.norm(self.current_direction) 
                

        angle_rot = self.rotation_angle(current_direction, self.current_direction)

        print("Rotate ", angle_rot)

        if abs(angle_rot)> 5:
            sign=1.0
            if angle_rot<0:
                sign=-1.0

            twist_msg.angular= Vector3(0,0,angle_rot/5)
        else:
            twist_msg.linear= Vector3(1,0,0)

        #Goal Seek and Wall Follow behavior implementation

        #Algo
        #get a line from start to goal

        #Head towards goal on the m-line

        #If an obstacle is in way, follow it until you encounter the m-line again

     
        #
        self.drive_pub.publish(twist_msg)




def main(args):
  
  rospy.init_node('bug2', anonymous=True)
  evade_obj = Bug2()
  rospy.spin()



if __name__=='__main__':
	main(sys.argv)