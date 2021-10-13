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


from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


from Line import Line

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
    def __init__(self, start_pt, goal_pt, car_state,collision_detection_threshold=1.5):
        self.mode = self.mode_list[0]
        self.goal_line = Line(start_pt, goal_pt)
        self.start_pt = start_pt
        self.goal_pt = goal_pt
        self.car_current_state = car_state
        self.last_change_pt = start_pt

        self.obstacle_lines = []

        self.collision_detection_threshold = collision_detection_threshold

        self.motion_state = self.motion_list[1]

        current_direction = self.car_current_state.get_direction_vector()
        self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
        #self.required_rotation = 0.0

        self.angle_alignment_threshold = 3.0

        #self.current_rotation_direction = current_direction

        self.rotation_angle = -90 #right

        

    def update_obstacle(self, obstacle_lines):
        # if self.mode == self.mode_list[1] and self.motion_state ==self.motion_list[1]:
        #     return
        self.obstacle_lines = obstacle_lines

    def is_colliding(self):
        ob_line_normal = self.get_obstacle_line()
        if ob_line_normal:
            obstacle_angle = self.get_obstacle_angle()
            
            dist_from_obstacle = ob_line_normal.mag()
            print(dist_from_obstacle,"obstacle dist")
            #dist_from_obstacle = self.get_obstacle_line().get_shortest_dist_from_pt(self.car_current_state.get_position_vector())
            if  dist_from_obstacle < self.collision_detection_threshold:
                print("collision detected  with angle  ", obstacle_angle)
                if abs(obstacle_angle) < 50.0:
                    print("collision detected ", obstacle_angle," dist", dist_from_obstacle)
                    return True

        return False

    def get_closest_obstacle_point(self):

        
        if len(self.obstacle_lines)>0:
            closest_dist = 10000.0
            best_line = self.obstacle_lines[0]

            origin = np.array([0.0,0.0])
            for line in self.obstacle_lines:
                dist = line.get_shortest_dist_from_pt(origin)
                if dist == closest_dist:
                    closest_dist = dist
                    best_line = line
            closest_obstacle_pt = best_line.get_closest_point_from_pt(origin)
            return closest_obstacle_pt
        
        return None

    def get_obstacle_line(self):
        closest_pt = self.get_closest_obstacle_point()
        
        if closest_pt is not None:
            return Line(np.array([0.0,0.0]), closest_pt)
        return None      
    def get_obstacle_angle(self):
        ob_line = self.get_obstacle_line()
        if ob_line:
            x_axis = Line(np.array([0.0,0.0]), np.array([1.0,0.0]))
            return ob_line.get_alignment_angle(x_axis)
            #return self.get_obstacle_line().get_angle_of_normal_at_orgin()
        return None
            

    def get_required_direction(self):
        ob_line = self.get_obstacle_line()
        if ob_line:
            x_axis = Line(np.array([0.0,0.0]), np.array([1.0,0.0]))
            ob_angle = ob_line.get_alignment_angle(x_axis)
            required_angle = ob_angle + self.rotation_angle
            #print(required_angle, "required angle ",ob_angle," ob angle")
            
            return self.get_obstacle_line().get_rotated_vector(self.rotation_angle)
        return self.goal_line

    def follow_wall(self):
        self.mode = self.mode_list[1] # wall follow state
        self.motion_state = self.motion_list[1] # need to rotate to allign parallel to wall

        #get rotation angle to align with the wall
        current_direction = self.car_current_state.get_direction_vector()

        self.required_rotation = self.get_required_direction().get_alignment_angle(current_direction)

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

        # if not self.is_colliding() and self.mode != self.mode_list[0]:
        #     if self.motion_state != self.motion_list[1]:
        #         self.follow_goal()
        if self.mode == self.mode_list[0]: #goal seeking
            if self.motion_state ==self.motion_list[0]: #Moving Forward
                
                if self.is_colliding(): #detected wall
                    
                    self.follow_wall()
                    linear_motion = [0,0,0]
                else:
                    # move forward, without rotation
                    #linear_motion = [0,0,0]
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
                    #self.current_rotation_direction = self.get_required_direction()
                    self.required_rotation = self.get_required_direction().get_alignment_angle(current_direction)
                    #print("wall following --------- with rotation",self.required_rotation )
                    linear_motion = [0.5,0,0]

            else: #Wall following started, need to rotate and align
                current_direction = self.car_current_state.get_direction_vector()
                #self.current_rotation_direction = self.get_required_direction()
                obstacle_angle = self.get_required_direction().get_alignment_angle(current_direction)
                obstacle_angle_current = self.get_obstacle_angle()
                print(obstacle_angle," --- Obstacle angle ", obstacle_angle_current,self.required_rotation)
                

                #if math.fabs(self.current_rotation_direction.get_alignment_angle(current_direction)) < self.angle_alignment_threshold and 
                if math.fabs(self.required_rotation) < 5:

                # if math.fabs(obstacle_angle) < self.angle_alignment_threshold and math.fabs(self.required_rotation) < 4: # rotation complete, need to move forward
                    self.required_rotation=0.0
                    #linear_motion = [0,0,0]
                    #self.motion_state = self.motion_list[0] #set motion to forward, as rotation is complete
                else:
                    #continue rotation as still not aligned with wall
                    linear_motion = [0,0,0]

                    #current_direction = self.car_current_state.get_direction_vector()
                    self.current_rotation_direction = self.get_required_direction()
                    self.required_rotation = self.get_required_direction().get_alignment_angle(current_direction)
                
        
        
        angular_motion =[0,0,self.required_rotation]

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

        drive_line_topic = rospy.get_param("~drive_line")

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
        self.m_line = Line(self.start_pt,self.goal_pt)
        
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.move)
        self.odom_sub = rospy.Subscriber(ransac_lines_topic, Marker, self.update_obstacle_detection)

        self.wall_line= None

        self.current_mode=mod_list[0]


        self.motion_algo = Bug2(self.start_pt,self.goal_pt, self.car_state)

        self.marker_pub = rospy.Publisher(drive_line_topic, Marker, queue_size=10)
  



    def update_obstacle_detection(self, msg):


        indx = 0
        lines = []
        for i in range(len(msg.points)/2):
            
            p1 = msg.points[indx*i]
            p2 = msg.points[indx*i+1]
            p1 = np.array([p1.x,p1.y])
            p2 = np.array([p2.x,p2.y])
            lines.append(Line(p1,p2))
            
        #self.wall_line = Line(p1,p2)

        self.motion_algo.update_obstacle(lines)

    def move(self,msg):
        #Get odometry and execute the algorithm
        #update car current state

        self.motion_algo.car_current_state.update_pose(msg)

        [linear, angular] = self.motion_algo.move()

        twist_msg = Twist()
        twist_msg.linear= Vector3(linear[0],0,0)
        

        
        rotation_val = 0.0
        if math.fabs(angular[2]) > 0.0:
            rotation_val=0.5
            if angular[2] < 0:
                rotation_val=rotation_val*-1


        twist_msg.angular= Vector3(0,0,rotation_val)
        print(linear[0],angular[2], self.motion_algo.mode, self.motion_algo.motion_state, self.motion_algo.get_obstacle_angle())
    

        self.drive_pub.publish(twist_msg)

        self.publish_lines([self.motion_algo.get_required_direction()])

    def publish_lines(self, lines):
        if len(lines)>0: # check for list is not empty
            fitted_lines = Marker()
            fitted_lines.type = Marker.LINE_LIST
            fitted_lines.scale.x=0.2
            fitted_lines.scale.y=0.2
            fitted_lines.color = ColorRGBA(0.0,1.0,0,1)   
            fitted_lines.header.frame_id= "odom"     
            for line in lines:
                [p1,p2] = line.get_ros_points()
                fitted_lines.points.append(p1)
                fitted_lines.points.append(p2)
            self.marker_pub.publish(fitted_lines)



def main(args):
  
  rospy.init_node('bug2', anonymous=True)
  evade_obj = Bug2Motion()
  rospy.spin()



if __name__=='__main__':
	main(sys.argv)