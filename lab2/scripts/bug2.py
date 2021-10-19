#!/usr/bin/env python
import sys

import rospy


from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Vector3


from visualization_msgs.msg import Marker

import numpy as np

import tf
import math

from std_msgs.msg import ColorRGBA

import time

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
    def __init__(self, start_pt, goal_pt, car_state,collision_detection_threshold=0.9, wall_follow_dist=1.1):
        self.mode = self.mode_list[0]
        self.goal_line = Line(start_pt, goal_pt)
        self.start_pt = start_pt
        self.goal_pt = goal_pt
        self.car_current_state = car_state
        self.last_change_pt = start_pt

        self.obstacle_lines = []

        self.collision_detection_threshold = collision_detection_threshold
        self.wall_follow_dist = wall_follow_dist
        self.motion_state = self.motion_list[1]

        current_direction = self.car_current_state.get_direction_vector()
        self.required_rotation = self.goal_line.get_alignment_angle(current_direction)

        self.angle_alignment_threshold = 3.0

        self.rotation_angle = -90 #right
        self.kp=0.1
        self.ki=0.1
        self.kd=0.1
        self.prev_error=None
        self.integral=0.0
        self.look_ahead=0.1
        self.last_time=None

        
    def get_goal_distance(self):
        return self.car_current_state.get_distance_from_point(self.goal_pt)

    def update_obstacle(self, obstacle_lines):
        self.obstacle_lines = obstacle_lines

    def front_collision_dist(self):
        ob_line_normal = self.get_obstacle_line()
        if ob_line_normal:
            obstacle_angle = self.get_obstacle_angle()
            
            dist_from_obstacle = ob_line_normal.mag()

            if  dist_from_obstacle < self.collision_detection_threshold:
                
                if abs(obstacle_angle) < 60.0:
                    return dist_from_obstacle

        return 10

    def dist_wall_on_left(self):
        #need to get the obstacle on left
        left_wall_dist = 10
        ob_line_normal = self.get_obstacle_line_between(0, 110) 
        if ob_line_normal:
            dist_from_obstacle = ob_line_normal.mag()
            if  dist_from_obstacle < 4:#self.wall_follow_dist:
                left_wall_dist = dist_from_obstacle
                return left_wall_dist

        return left_wall_dist        

    def get_closest_obstacle_point(self):
        
        if len(self.obstacle_lines)>0:
            closest_dist = 10000.0
            best_line = self.obstacle_lines[0]

            origin = np.array([0.0,0.0])
            for line in self.obstacle_lines:
                dist = line.get_shortest_dist_from_pt(origin)
                if dist < closest_dist:
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

    def get_obstacle_line_between(self, start_angle, end_angle):
        
        best_line=None
        max_dist = 0
        if len(self.obstacle_lines)>0:
            closest_dist = 10000.0
            

            origin = np.array([0.0,0.0])
            for line in self.obstacle_lines:
                dist = line.get_shortest_dist_from_pt(origin)
                closest_obstacle_pt = line.get_closest_point_from_pt(origin)
                ob_line = Line(np.array([0.0,0.0]), closest_obstacle_pt)
                x_axis = Line(np.array([0.0,0.0]), np.array([1.0,0.0]))
                angle_on_x = ob_line.get_alignment_angle(x_axis)
                if start_angle <= angle_on_x and end_angle >=angle_on_x:
                    if dist < closest_dist:
                        closest_dist = dist
                        best_line = ob_line

        return best_line

    def get_obstacle_angle(self):
        ob_line = self.get_obstacle_line()
        if ob_line:
            x_axis = Line(np.array([0.0,0.0]), np.array([1.0,0.0]))
            return ob_line.get_alignment_angle(x_axis)
            #return self.get_obstacle_line().get_angle_of_normal_at_orgin()
        return None
          
    def get_required_direction(self, rotation_angle):

        if self.mode == self.mode_list[1]: # wall following
            ob_line = self.get_obstacle_line()
            if ob_line:
                x_axis = Line(np.array([0.0,0.0]), np.array([1.0,0.0]))
                ob_angle = ob_line.get_alignment_angle(x_axis)
                required_angle =rotation_angle -  ob_angle
                return ob_line.get_rotated_vector(required_angle)
            return None
        return self.goal_line

    def follow_wall(self):
        self.mode = self.mode_list[1] # wall follow state
        self.motion_state = self.motion_list[1] # need to rotate to allign parallel to wall

        #get rotation angle to align with the wall
        current_direction = self.car_current_state.get_direction_vector()
        
        rotated_direction = self.get_required_direction(self.rotation_angle)
        self.required_rotation =0.0
        if rotated_direction:
            self.required_rotation = rotated_direction.get_alignment_angle(current_direction)

        self.last_change_pt = self.car_current_state.get_position_vector()

    def follow_goal(self):
        self.mode = self.mode_list[0] # goal follow state
        self.motion_state = self.motion_list[1] # need to rotate to allign parallel to goal line

        #get rotation angle to align with the goal line
        current_direction = self.car_current_state.get_direction_vector()
        self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
                
        self.last_change_pt = None 

    def is_wall_following(self):
        return self.mode == self.mode_list[1] and self.motion_state ==self.motion_list[0]

    def goal_seek(self):
        current_direction = self.car_current_state.get_direction_vector()
               
        linear_motion = [0.5,0,0]
        angular_motion =[0,0,0]

        current_direction = self.car_current_state.get_direction_vector()

        goal_dist = self.get_goal_distance()
        if goal_dist <0.1:
            print("goal Reached..........")
            linear_motion = [0,0,0]
            self.required_rotation = 0

        elif goal_dist < 0.7:
            print("goal reaching..........")
            linear_motion = [0.08,0,0]
            self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
            augular_velocity = 0.4
            if self.required_rotation < 0:
                augular_velocity*=-1
            angular_motion =[0,0,augular_velocity]

            return [linear_motion,angular_motion]

        elif self.motion_state ==self.motion_list[0]: #Moving Forward
            
            if self.front_collision_dist() < 2: #detected wall                    
                self.follow_wall()
                linear_motion = [0,0,0]
            else:
                # move forward, without rotation
                linear_motion = [1,0,0]
        else: #Goal following started, need to rotate and align with goal line
            if math.fabs(self.goal_line.get_alignment_angle(current_direction)) < self.angle_alignment_threshold and math.fabs(self.required_rotation) < 3: # rotation complete, need to move forward
                self.required_rotation=0.0
                self.motion_state = self.motion_list[0] #set motion to forward, as rotation is complete
            else:
                #continue rotation as still not aligned with goal
                self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
                linear_motion = [0,0,0]
        augular_velocity=self.required_rotation
        if self.required_rotation > 0:
            augular_velocity = 0.4
        elif self.required_rotation < 0:
            augular_velocity = -0.4
        
        angular_motion = [0,0,augular_velocity]

        return [linear_motion,angular_motion]

    def wall_follow(self):
        current_direction = self.car_current_state.get_direction_vector()
               

        linear_motion = [0.7,0,0]
        angular_motion =[0,0,0]

        if self.motion_state ==self.motion_list[0]: #Moving Forward
            
            d = self.goal_line.get_shortest_dist_from_pt(self.car_current_state.get_position_vector())
            
            ignore_point = False
            if self.last_change_pt is not None:
                dist_val = self.car_current_state.get_distance_from_point(self.last_change_pt)
                ignore_point = dist_val < 0.7
            if d < 0.08 and not ignore_point: #approaching goal line
                self.follow_goal()
                linear_motion = [0,0,0]
            else:
                #continue moving parallel to wall 
                #TODO: need to use PID for wall follow
                wall_dist = self.dist_wall_on_left()
                
                if wall_dist > self.wall_follow_dist*0.9:
                    self.required_rotation = 1
                    linear_motion = [0.7,0,0]

                elif wall_dist <= self.wall_follow_dist*0.5:
                    linear_motion = [0.5,0,0]
                    
                    self.required_rotation = self.get_required_direction(self.rotation_angle).get_alignment_angle(current_direction)
                else:

                    linear_motion = [0.8,0,0]
                    self.required_rotation = 0.0
                
            
            wall_dist = self.dist_wall_on_left()
            return self.pid_control(wall_dist,0.4)
        else: #Wall following started, need to rotate and align
            current_direction = self.car_current_state.get_direction_vector()
            
            obstacle_angle = self.get_required_direction(self.rotation_angle).get_alignment_angle(current_direction)
            obstacle_angle_current = self.get_obstacle_angle()
            
            if obstacle_angle_current > 85:
                self.required_rotation=0.0
                self.motion_state = self.motion_list[0] #set motion to forward, as rotation is complete
            else:
                #continue rotation as still not aligned with wall
                linear_motion = [0.3,0,0]
                self.required_rotation = obstacle_angle
        augular_velocity=self.required_rotation
        if self.required_rotation > 0:
            augular_velocity = 1.5
        elif self.required_rotation < 0:
            augular_velocity = -1.5



        angular_motion = [0,0,augular_velocity]
        return [linear_motion,angular_motion]

    def pid_control(self, curr_dist, velocity):

        now = time.time()
        p_error=0
        d_error=0
        i_error=0
        current_direction = self.car_current_state.get_direction_vector()
        #angle = self.get_required_direction(self.rotation_angle).get_alignment_angle(current_direction)
        ob_Line = self.get_obstacle_line_between(-30, 120) 


        if ob_Line:
            x_axis = Line(np.array([0.0,0.0]), np.array([1.0,0.0]))
            angle = ob_Line.get_alignment_angle(x_axis)
            angle = angle-90
            angle = math.radians(angle)
            wall_dist=ob_Line.mag()
        else:
            wall_dist=5
            angle = math.degrees(90)
            self.integral=0
        #angle = self.get_obstacle_angle()
        # wall_dist = self.wall_follow_dist
        # if angle is None:
        #     angle=-1.5
        #     print("wall not found")
        # else:
        #    angle = angle-90
        #    #print("wall angle", angle)
        #    angle = math.radians(angle)
        # ob_line = self.get_obstacle_line()
        # if ob_line:
        #     wall_dist = ob_line.mag()
        # else:
        #     wall_dist =4

        
        d_t = wall_dist#* math.cos(angle)
        d_1 = d_t + self.look_ahead*math.sin(angle)
        
        

        error = d_1 - self.wall_follow_dist# - d_1

        print(wall_dist)
        #error = curr_dist
        if self.last_time:
            dt = now -self.last_time
            self.integral += self.ki * error * dt
            i_error= self.integral
            if self.prev_error:
               d_error = self.kd*error*dt

        p_error = self.kp*error 
                

        #TODO: Use kp, ki & kd to implement a PID controller for 




        error_out = p_error+d_error+i_error


        
        self.prev_error = error
        self.last_time = now

        linear_motion = [velocity,0,0]
        angular_motion =[0,0,error_out]
        return [linear_motion,angular_motion]

    def move(self):
        #need to return linear and angular velocities


        #Goal Seak
        if self.mode == self.mode_list[0]: #goal seeking
            [linear_motion,angular_motion] = self.goal_seek()

        elif self.mode == self.mode_list[1]: #wall following
            [linear_motion,angular_motion] = self.wall_follow()
        else:
            print("reached goal")
            linear_motion = [0,0,0]
            angular_motion =[0,0,0]

        #print(self.mode, self.motion_state, linear_motion,angular_motion)
        return [linear_motion,angular_motion]

        linear_motion = [0.5,0,0]
        current_direction = self.car_current_state.get_direction_vector()

        goal_dist = self.get_goal_distance()
        if goal_dist <0.1:
            print("goal Reached..........")
            linear_motion = [0,0,0]
            self.required_rotation = 0

        elif goal_dist < 0.7:
            print("goal reaching..........")
            linear_motion = [0.08,0,0]
            self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
            augular_velocity = 0.4
            if self.required_rotation < 0:
                augular_velocity*=-1
            angular_motion =[0,0,augular_velocity]

            return [linear_motion,angular_motion]

        elif self.mode == self.mode_list[0]: #goal seeking
            if self.motion_state ==self.motion_list[0]: #Moving Forward
                
                if self.front_collision_dist() < 2: #detected wall                    
                    self.follow_wall()
                    linear_motion = [0,0,0]
                else:
                    # move forward, without rotation
                    linear_motion = [1,0,0]
            else: #Goal following started, need to rotate and align with goal line
                if math.fabs(self.goal_line.get_alignment_angle(current_direction)) < self.angle_alignment_threshold and math.fabs(self.required_rotation) < 3: # rotation complete, need to move forward
                    self.required_rotation=0.0
                    self.motion_state = self.motion_list[0] #set motion to forward, as rotation is complete
                else:
                    #continue rotation as still not aligned with goal
                    self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
                    linear_motion = [0,0,0]

        elif self.mode == self.mode_list[1]: #Wall following
            if self.motion_state ==self.motion_list[0]: #Moving Forward
                
                d = self.goal_line.get_shortest_dist_from_pt(self.car_current_state.get_position_vector())
                
                ignore_point = False
                if self.last_change_pt is not None:
                    dist_val = self.car_current_state.get_distance_from_point(self.last_change_pt)
                    ignore_point = dist_val < 0.7
                if d < 0.08 and not ignore_point: #approaching goal line
                    self.follow_goal()
                    linear_motion = [0,0,0]
                else:
                    #continue moving parallel to wall 
                    #TODO: need to use PID for wall follow
                    wall_dist = self.dist_wall_on_left()
                    
                    if wall_dist > self.wall_follow_dist*0.9:
                        self.required_rotation = 1
                        linear_motion = [0.7,0,0]

                    elif wall_dist <= self.wall_follow_dist*0.5:
                        linear_motion = [0.5,0,0]
                        
                        self.required_rotation = self.get_required_direction(self.rotation_angle).get_alignment_angle(current_direction)
                    else:

                        linear_motion = [0.8,0,0]
                        self.required_rotation = 0.0

            else: #Wall following started, need to rotate and align
                current_direction = self.car_current_state.get_direction_vector()
                
                obstacle_angle = self.get_required_direction(self.rotation_angle).get_alignment_angle(current_direction)
                obstacle_angle_current = self.get_obstacle_angle()
                
                if obstacle_angle_current > 85:
                    self.required_rotation=0.0
                    self.motion_state = self.motion_list[0] #set motion to forward, as rotation is complete
                else:
                    #continue rotation as still not aligned with wall
                    linear_motion = [0.3,0,0]
                    self.required_rotation = obstacle_angle
        
        augular_velocity=self.required_rotation
        if self.required_rotation > 0:
            augular_velocity = 0.4
        elif self.required_rotation < 0:
            augular_velocity = -0.4

        if self.mode == self.mode_list[1]:
            augular_velocity*=1.5

        if self.is_wall_following():
            augular_velocity*=0.9
        
            
        angular_motion =[0,0,augular_velocity]

        return [linear_motion,angular_motion]




class Bug2Motion:
    """ Implement ransac algo to display lines
    """
    def __init__(self):

        #new Implementation
        drive_topic = rospy.get_param("~goal_seeker_drive_topic")
        odom_topic = rospy.get_param("~odom_topic")

        start_pt = np.array(rospy.get_param("~start_pt"))
        goal_pt = np.array(rospy.get_param("~goal_pt"))

        ransac_lines_topic = rospy.get_param("~ransac_lines")

        self.drive_pub = rospy.Publisher(drive_topic, Twist,queue_size=10)

        car_state = CarState([start_pt[0],start_pt[1],90.0])
        
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.move)

        self.marker_sub = rospy.Subscriber(ransac_lines_topic, Marker, self.update_obstacle_detection)


        collision_detection_threshold= rospy.get_param("~collision_detection_threshold")

        wall_follow_dist= rospy.get_param("~wall_follow_dist")

        kp = rospy.get_param("~kp")
        ki = rospy.get_param("~ki")
        kd = rospy.get_param("~kd")
        look_ahead = rospy.get_param("~look_ahead")


        self.motion_algo = Bug2(start_pt, goal_pt, car_state,collision_detection_threshold,wall_follow_dist)
        self.motion_algo.kp = kp
        self.motion_algo.ki = ki
        self.motion_algo.kd = kd
        self.look_ahead = look_ahead


    def update_obstacle_detection(self, msg):
        indx = 2
        lines = []
        for i in range(len(msg.points)/2):
            
            p1 = msg.points[indx*i]
            p2 = msg.points[indx*i+1]
            p1 = np.array([p1.x,p1.y])
            p2 = np.array([p2.x,p2.y])
            lines.append(Line(p1,p2))

        self.motion_algo.update_obstacle(lines)

    def move(self,msg):
        #Get odometry and execute the algorithm
        #update car current state
        self.motion_algo.car_current_state.update_pose(msg)

        [linear, angular] = self.motion_algo.move()
        
        twist_msg = Twist()
        twist_msg.linear= Vector3(linear[0],0,0)
        rotation_val = angular[2]
        
        twist_msg.angular= Vector3(0,0,rotation_val)    

        self.drive_pub.publish(twist_msg)


def main(args):
  
  rospy.init_node('bug2', anonymous=True)
  evade_obj = Bug2Motion()
  rospy.spin()



if __name__=='__main__':
	main(sys.argv)