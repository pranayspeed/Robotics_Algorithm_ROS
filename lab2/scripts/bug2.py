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


class Line:
    #initialize goal line with start and end point
    def __init__(self, start_pt, end_pt):
        self.start = start_pt
        self.end = end_pt

    def get_shortest_dist_from_pt(self, pt):
        return abs(np.cross(self.end-self.start, self.start-pt)/np.linalg.norm(self.end-self.start))

    
    def get_alignment_angle(self, other_line):
        # print(other_line.end)
        # print(other_line.start)
        # print(self.end)
        # print(self.start)
        vec1 = other_line.end - other_line.start
        vec2 = self.end - self.start
        return math.degrees(math.atan2(vec2[1],vec2[0]) - math.atan2(vec1[1],vec1[0]))
        

    def get_angle_of_normal_at_orgin(self):

        pt = np.array([0.0,0.0])
        x_axis = Line(np.array(pt), np.array([1.0,0.0]))
        normal_vec_line  = self.get_normal_from_pt(pt)
        normal_angle = normal_vec_line.get_alignment_angle(x_axis)

        return normal_angle
        #dist = np.linalg.norm(np.array(pt) - pt_on_line)
        #print("goal line distance", dist)

    def get_normal_from_pt(self, pt):
        vec = self.end-self.start
        
        uvec = vec / np.linalg.norm(vec)
        pt_vec = pt - self.start
        dist_proj = np.dot(pt_vec, uvec)
        proj_vector = uvec * dist_proj
        pt_on_line = self.start + proj_vector
        
        normal_vec = np.array(pt) - pt_on_line
        return Line(np.array(pt), pt_on_line)

class CarState:
    def __init__(self, states):
        self.x = states[0]
        self.y = states[1]
        self.theta = states[2]

    def update_pose(self, odom_msg):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2] #Yaw        
    
    def get_direction_vector(self):
        p1 = np.array([self.x,self.y])
        p2 = np.array([self.x + math.cos(self.theta),self.y+math.sin(self.theta)])
        return Line(p1,p2)
    
    def get_position_vector(self):
        return np.array([self.x,self.y])

    def get_distance_from_point(self, pt):
        vec = self.get_position_vector() - pt

        return np.sqrt(np.dot(vec.T, vec))

class Bug2:
    mode_list = ["GOAL_SEEK","WALL_FOLLOW"]
    motion_list = ["forward", "rotate"]
    def __init__(self, start_pt, goal_pt, car_state,collision_detection_threshold=6):
        self.mode = self.mode_list[0]
        self.goal_line = Line(start_pt, goal_pt)
        self.start_pt = start_pt
        self.goal_pt = goal_pt
        self.car_current_state = car_state
        self.last_change_pt = start_pt

        self.obstacle_line = None

        self.collision_detection_threshold = collision_detection_threshold

        self.motion_state = self.motion_list[1]

        current_direction = self.car_current_state.get_direction_vector()
        self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
        #self.required_rotation = 0.0

        self.angle_alignment_threshold = 3.0

        self.current_rotation_direction = current_direction

        

    def update_obstacle(self, obstacle_line):
        if self.mode == self.mode_list[1] and self.motion_state ==self.motion_list[1]:
            return
        self.obstacle_line = obstacle_line

    def is_colliding(self):
        if self.obstacle_line:
            obstacle_angle = self.get_obstacle_angle()
            
            dist_from_obstacle = self.obstacle_line.get_shortest_dist_from_pt(self.car_current_state.get_position_vector())
            print(obstacle_angle, dist_from_obstacle)
            if  dist_from_obstacle < self.collision_detection_threshold:
                print("collision detected  with angle  ", obstacle_angle)
                if abs(obstacle_angle) < 45.0:
                    print("collision detected ", obstacle_angle)
                    return True
        return False

    def get_obstacle_angle(self):
        if self.obstacle_line:
            return self.obstacle_line.get_angle_of_normal_at_orgin()
        return None
            

    def get_current_direction(self):
        return self.obstacle_line.get_normal_from_pt(np.array([0.0,0.0]))

    def follow_wall(self):
        self.mode = self.mode_list[1] # wall follow state
        self.motion_state = self.motion_list[1] # need to rotate to allign parallel to wall

        #get rotation angle to align with the wall
        current_direction = self.car_current_state.get_direction_vector()
        #self.required_rotation = self.obstacle_line.get_alignment_angle(current_direction)
        #self.required_rotation = self.get_obstacle_angle() - 90
        self.current_rotation_direction = self.get_current_direction()

        self.required_rotation = self.current_rotation_direction.get_alignment_angle(current_direction)

        self.last_change_pt = self.car_current_state.get_position_vector()

    def follow_goal(self):
        self.mode = self.mode_list[0] # goal follow state
        self.motion_state = self.motion_list[1] # need to rotate to allign parallel to goal line

        #get rotation angle to align with the goal line
        current_direction = self.car_current_state.get_direction_vector()
        self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
        
        
        self.last_change_pt = None 
        #self.car_current_state.get_position_vector()

    def is_wall_following(self):
        return self.mode == self.mode_list[1] and self.motion_state ==self.motion_list[0]

    def move(self):
        #need to return linear and angular velocities
        linear_motion = [0.5,0,0]

        
        if self.mode == self.mode_list[0]: #goal seeking
            if self.motion_state ==self.motion_list[0]: #Moving Forward
                
                if self.is_colliding(): #detected wall
                    self.follow_wall()
                    linear_motion = [0,0,0]
                else:
                    # move forward, without rotation
                    print("Moving forward without collision at front")
            else: #Goal following started, need to rotate and align with goal line
                current_direction = self.car_current_state.get_direction_vector()
                if math.fabs(self.goal_line.get_alignment_angle(current_direction)) < self.angle_alignment_threshold and math.fabs(self.required_rotation) < 3: # rotation complete, need to move forward
                    self.required_rotation=0.0
                    self.motion_state = self.motion_list[0] #set motion to forward, as rotation is complete
                else:
                    #continue rotation as still not aligned with goal
                    current_direction = self.car_current_state.get_direction_vector()
                    self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
                    linear_motion = [0,0,0]

        elif self.mode == self.mode_list[1]: #Wall following
            if self.motion_state ==self.motion_list[0]: #Moving Forward
                d = self.goal_line.get_shortest_dist_from_pt(self.car_current_state.get_position_vector())
                if d < 0.0001: #approaching goal line
                    self.follow_goal()
                    linear_motion = [0,0,0]
                else:
                    #pass #continue moving parallel to wall 
                    #TODO: need to use PID for wall follow
                    current_direction = self.car_current_state.get_direction_vector()
                    self.current_rotation_direction = self.get_current_direction()
                    self.required_rotation = self.current_rotation_direction.get_alignment_angle(current_direction)
                    print("wall following --------- with rotation",self.required_rotation )
                    linear_motion = [0.1,0,0]

            else: #Wall following started, need to rotate and align
                current_direction = self.car_current_state.get_direction_vector()
                self.current_rotation_direction = self.get_current_direction()
                obstacle_angle = self.current_rotation_direction.get_alignment_angle(current_direction)
                #obstacle_angle = self.get_obstacle_angle()
                print(obstacle_angle," --- Obstacle angle ", self.required_rotation)


                #if math.fabs(self.current_rotation_direction.get_alignment_angle(current_direction)) < self.angle_alignment_threshold and 
                if math.fabs(self.required_rotation) < 3:

                # if math.fabs(obstacle_angle) < self.angle_alignment_threshold and math.fabs(self.required_rotation) < 4: # rotation complete, need to move forward
                    self.required_rotation=0.0
                    self.motion_state = self.motion_list[0] #set motion to forward, as rotation is complete
                else:
                    #continue rotation as still not aligned with wall
                    linear_motion = [0,0,0]

                    current_direction = self.car_current_state.get_direction_vector()
                    self.current_rotation_direction = self.get_current_direction()
                    self.required_rotation = self.current_rotation_direction.get_alignment_angle(current_direction)
                
        
        
        angular_motion =[0,0,self.required_rotation]

        # if self.required_rotation !=0.0:
        #     linear_motion = [0,0,0]

        return [linear_motion,angular_motion]

                    

                    
            





mod_list = ["GOAL_SEEK","WALL_FOLLOW"]
class Bug2Motion:
    """ Implement ransac algo to display lines
    """
    def __init__(self):
        

        drive_topic = rospy.get_param("~goal_seeker_drive_topic")
        odom_topic = rospy.get_param("~odom_topic")


        self.start_pt = np.array(rospy.get_param("~start_pt"))
        self.goal_pt = np.array(rospy.get_param("~goal_pt"))

        ransac_lines_topic = rospy.get_param("~ransac_lines")



        self.goal_vec = self.goal_pt - self.start_pt




        self.drive_pub = rospy.Publisher(drive_topic, Twist,queue_size=10)

        self.current_state=mod_list[0]

        self.current_wall=None
        self.current_direction=self.goal_vec

        self.collision_threshold = rospy.get_param("~collision_threshold")
        self.rotating=False


        #new Implementation

        self.car_state = None 
        self.car_state = CarState([self.start_pt[0],self.start_pt[1],90.0])
        self.m_line = Line(np.array(self.start_pt),np.array(self.goal_pt))
        
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.move)
        self.odom_sub = rospy.Subscriber(ransac_lines_topic, Marker, self.update_obstacle_detection)

        self.wall_line= None

        self.current_mode=mod_list[0]


        self.motion_algo = Bug2(self.start_pt,self.goal_pt, self.car_state)

    def update_obstacle_detection(self, msg):
        p1 = msg.points[0]
        p2 = msg.points[1]
        p1 = np.array([p1.x,p1.y])
        p2 = np.array([p2.x,p2.y])
        self.wall_line = Line(p1,p2)

        self.motion_algo.update_obstacle(Line(p1,p2))

    def move(self,msg):
        #Get odometry and execute the algorithm
        #update car current state

        #self.car_state.update_pose(msg)
        self.motion_algo.car_current_state.update_pose(msg)

        [linear, angular] = self.motion_algo.move()

        twist_msg = Twist()
        twist_msg.linear= Vector3(linear[0],0,0)
        

        rotation_val = 0.0
        if math.fabs(angular[2]) >0.0:
            rotation_val=1.0
            if angular[2] < 0:
                rotation_val=rotation_val*-1
        # if self.motion_algo.is_wall_following():#Increase the rotation speed
        #     rotation_val=rotation_val*45.0

        twist_msg.angular= Vector3(0,0,rotation_val)
        print(linear[0],angular[2], self.motion_algo.mode, self.motion_algo.motion_state, self.motion_algo.get_obstacle_angle())

        # twist_msg.linear= Vector3(0,0,0)        
        # twist_msg.angular= Vector3(0,0,0)       

        self.drive_pub.publish(twist_msg)
            




        









    # def update_wall(self, msg):
        
    #     p1 = msg.points[0]
    #     p2 = msg.points[1]
    #     p1 = np.array([p1.x,p1.y])
    #     p2 = np.array([p2.x,p2.y])
    #     d = abs(np.cross(p2-p1, p1)/np.linalg.norm(p2-p1))

    #     # if self.current_state == state_list[0]:
    #     #     self.current_wall=None
    #     if d < self.collision_threshold:
    #         self.current_wall=[p1,p2]
    #         print("Wall found")


  
    # def check_if_encountered_goal_line(self, pt):

    #     goal_unit_vec = self.goal_vec / np.linalg.norm(self.goal_vec)         # unit vector from r0 to r1
    #     pt_vec = np.array(pt) - np.array(self.start_pt)             # vector from r0 to pt
    #     dist_proj = np.dot(pt_vec, goal_unit_vec)   # projection (length) of r onto r01u
    #     proj_vector = goal_unit_vec * dist_proj         # projection vector
    #     pt_on_line = self.start_pt + proj_vector           # point on line
        
    #     dist = np.linalg.norm(np.array(pt) - pt_on_line)
    #     print("goal line distance", dist)
    #     return abs(dist) < 0.00001

    # def rotation_angle(self, line1, line2):
    #     return math.degrees(math.atan2(line2[1],line2[0]) - math.atan2(line1[1],line1[0]))

       

    # def check_if_collison(self):
    #     if self.current_wall:
    #         return True
    #     return False

    # def get_pose(self,msg):
    #     state_x = msg.pose.pose.position.x
    #     state_y = msg.pose.pose.position.y
        
    #     quaternion = (
    #         msg.pose.pose.orientation.x,
    #         msg.pose.pose.orientation.y,
    #         msg.pose.pose.orientation.z,
    #         msg.pose.pose.orientation.w)
    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     current_angle = euler[2] #Yaw

    #     return [state_x,state_y, current_angle]

    # def goal_walker(self, msg):
        
    #     twist_msg = Twist()
    #     twist_msg.linear= Vector3(0,0,0)
    #     twist_msg.angular= Vector3(0,0,0)


    #     current_angle=0

    #     current_direction = np.array([math.cos(current_angle),math.sin(current_angle)])

    #     if self.current_state == mod_list[0]:
    #         #Goal seeking
    #         print("Goal Seeking")
    #         #check collision
    #         if self.check_if_collison():
    #             #change state to wall follow direction parallel to wall
    #             self.current_state = mod_list[1]
                
    #             #get angle between obstacle and current direction and rotate
    #             self.current_direction = np.array(self.current_wall[0]) - np.array(self.current_wall[1]) 

    #             self.current_direction = self.current_direction / np.linalg.norm(self.current_direction) 
                
                
    #     else:
    #         # wall Following
    #         print("Wall following")
    #         #check if intersecting m-line
    #         if self.check_if_encountered_goal_line([state_x,state_y]) and not self.rotating:                
    #             #change state to goal seeking
    #             self.current_state = mod_list[0]
                
    #             #get angle between m_line and current direction and rotate

    #             self.current_direction = np.array([self.goal_pt[0],self.goal_pt[1]])- np.array([state_x,state_y])
    #             self.current_direction = self.current_direction / np.linalg.norm(self.current_direction) 
                 

    #     angle_rot = self.rotation_angle(current_direction, self.current_direction)

    #     print("Rotate ", angle_rot)

    #     if abs(angle_rot)> 3:
    #         sign=1.0
    #         if angle_rot<0:
    #             sign=-1.0

    #         twist_msg.angular= Vector3(0,0,sign*0.5)
    #         self.rotating=True
    #     else:
    #         twist_msg.linear= Vector3(0.5,0,0)
    #         self.rotating=False

    #     #
    #     self.drive_pub.publish(twist_msg)




def main(args):
  
  rospy.init_node('bug2', anonymous=True)
  evade_obj = Bug2Motion()
  rospy.spin()



if __name__=='__main__':
	main(sys.argv)