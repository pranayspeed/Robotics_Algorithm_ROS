#!/usr/bin/env python
import sys

import rospy

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Vector3


import numpy as np

import tf
import math


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
    
    def is_ahead(self, current_line):
        return self.get_distance_from_point(current_line.start) > current_line.mag()


class CarMotion:

    def __init__(self, start_pt, goal_pt, car_state):

        self.goal_line = Line(start_pt, goal_pt)
        self.start_pt = start_pt
        self.goal_pt = goal_pt
        self.car_current_state = car_state

        self.prev_error=None
        self.integral=0.0
        self.last_time=None


        
    def get_goal_distance(self):
        return self.car_current_state.get_distance_from_point(self.goal_pt)


    def pid_control(self, error, kp,ki,kd):

        now = time.time()
        p_error=0
        d_error=0
        i_error=0

        if self.last_time:
            dt = now -self.last_time
            self.integral += ki * error * dt
            i_error= self.integral
            if self.prev_error:
               d_error = kd*error*dt

        p_error = kp*error 

        error_out = p_error+d_error+i_error
      
        self.prev_error = error
        self.last_time = now

        return error_out


 
    def move(self, current_line):
               
        linear_motion = [0.2,0,0]
        angular_motion =[0,0,0]

        if current_line is None:
            curr_pos = self.car_current_state.get_position_vector()
            current_line = Line(curr_pos, self.goal_pt)

        current_direction = self.car_current_state.get_direction_vector()

        required_rotation = current_line.get_alignment_angle(current_direction)
        if required_rotation > 180:
            required_rotation = required_rotation - 360

        goal_dist = self.get_goal_distance()
        if goal_dist <0.5:
            if goal_dist <0.1:
                print("goal Reached..........")
                linear_motion = [0,0,0]
                required_rotation = 0

            elif goal_dist < 0.5:
                print("goal reaching..........")
                kp=0.2
                ki=0 
                kd=0.1        
                linear_motion[0] = self.pid_control(goal_dist, kp,ki,kd) 

            kp=0.09
            ki=0 
            kd=0.01  

        else:                
            if math.fabs(required_rotation) < 0.1:
                # move forward, without rotation
                linear_motion = [1,0,0]
            else:
                kp=0.05
                ki=0 
                kd=0.01  
                linear_motion[0] = self.pid_control(linear_motion[0], kp,ki,kd)

            kp=0.05
            ki=0 
            kd=0.01        
        augular_velocity = self.pid_control(required_rotation, kp,ki,kd)                
                
        angular_motion = [0,0,augular_velocity]

        return [linear_motion,angular_motion]

def read_map():
    map_file = rospy.get_param('map_file')
    with open(map_file) as fd:
        map_str = fd.read()
        map_str = map_str.split('[')[1].split(']')[0]
        map_rows = map_str.split('\n')
        map = []

        for row in map_rows:
            map_current_row = []
            for val in row.split(','):
                val_str = val.strip()
                if len(val_str)>0:
                    map_current_row.append(int(val_str))
            map.append(map_current_row)
    return map
def get_goal_path(start_pt, goal_pt, map):
    xo = len(map[0])/2
    yo = len(map)/2

    sx = start_pt[0]
    sy = start_pt[1]
    gx = goal_pt[0]
    gy = goal_pt[1]

    astar_planner = AStarPlanner(map)
    start = Node(xo + sx, yo - sy)
    goal = Node(xo + gx, yo - gy) 

    final_path = astar_planner.search(start, goal, euclidean_dist)

    new_path = []
    if final_path:
        for pt in final_path:
            new_path.append([ pt[0] - xo, - (pt[1] -yo)])
    print(new_path)
    return new_path

class AStar:
    """ Implement A* algo to find path to goal
    """
    def __init__(self):

        self.curr_index = 0

        drive_topic = rospy.get_param("~goal_seeker_drive_topic")
        odom_topic = rospy.get_param("~odom_topic")

        start_pt = np.array(rospy.get_param("~start_pt"))
        goal_pt = np.array(rospy.get_param("~goal_pt"))
        goal_orient = rospy.get_param("~orient")

        self.map = read_map()
        self.path_list = get_goal_path(start_pt, goal_pt, self.map)

        
        self.drive_pub = rospy.Publisher(drive_topic, Twist,queue_size=10)

        car_state = CarState([start_pt[0],start_pt[1],goal_orient])
        
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.update_pos)


        self.motion_algo = CarMotion(start_pt, goal_pt, car_state)


    def stop_move(self):
        if self.curr_index > len(self.path_list)-1:
            twist_msg = Twist()
            twist_msg.linear= Vector3(0,0,0)            
            twist_msg.angular= Vector3(0,0,0)    
            self.drive_pub.publish(twist_msg)
            return True
        return False

    def is_goal_updated(self):
        if rospy.has_param("goalx") and rospy.has_param("goaly"):
            goalx = rospy.get_param("goalx")
            goaly = rospy.get_param("goaly")
            goal_pt = self.motion_algo.goal_pt
            if goalx != goal_pt[0] or goaly != goal_pt[1]:
                start_pt = self.motion_algo.car_current_state.get_position_vector()
                goal_pt = [goalx, goaly]
                self.path_list = get_goal_path(start_pt, goal_pt, self.map)
                if len(self.path_list) <1:
                    print("cannot find path, please provide valid goal")
                    return True

                goal_orient = self.motion_algo.car_current_state.theta
                car_state = CarState([start_pt[0],start_pt[1],goal_orient])
                self.motion_algo = CarMotion(start_pt, goal_pt, car_state)


                self.curr_index= 0
                print("New goal updated")
                return True
        return False


    def update_pos(self,msg):
        #Get odometry and execute the algorithm
        #update car current state
    
        self.motion_algo.car_current_state.update_pose(msg)
        
        if self.is_goal_updated():
            return
        
        ci = self.curr_index

        current_line = None
        p1_curr = self.motion_algo.car_current_state.get_position_vector()

        if ci+1 < len(self.path_list):
            p1 = np.array(self.path_list[ci])
            p2 = np.array(self.path_list[ci+1])

            p1 = np.array([p1[0]+0.5,p1[1]-0.5])
            p2 = np.array([p2[0]+0.5,p2[1]-0.5])

            current_line = Line(p1,p2)

            if self.motion_algo.car_current_state.is_ahead(current_line):
                
                self.curr_index+=1
                if not self.stop_move():                
                    ci = self.curr_index
                    if ci+1 < len(self.path_list):
                        p1 = np.array(self.path_list[ci])
                        p2 = np.array(self.path_list[ci+1])
                        p1 = np.array([p1[0]+0.5,p1[1]-0.5])
                        p2 = np.array([p2[0]+0.5,p2[1]-0.5])
                        current_line = Line(p1,p2)
          
        
        [linear, angular] = self.motion_algo.move(current_line)
        
        twist_msg = Twist()
        twist_msg.linear= Vector3(linear[0],0,0)
        rotation_val = angular[2]
        
        twist_msg.angular= Vector3(0,0,rotation_val)    

        self.drive_pub.publish(twist_msg)




class Node:
    def __init__(self,x,y):
        self.x=int(x)
        self.y=int(y)

    def __eq__(self, other):
        return self.x == other.x and self.y==other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __str__(self):
        return str(self.x) +", "+str(self.y)
    def __repr__(self):
        return "(" + str(self.x) +", "+str(self.y) +")" 

def return_path(cameFrom, current):
    print("returning path")
    total_path = [[current.x, current.y]]
    while current in cameFrom:
        current = cameFrom[current]
        total_path.insert(0,[current.x, current.y])
    return total_path

def euclidean_dist(current,goal):
    return (((goal.x - current.x )**2) + ((goal.y-current.y)**2) )


# A* finds a path from start to goal.
# h is the heuristic function. h(n) estimates the cost to reach goal from node n.

class AStarPlanner:
    neighbours = [[-1,0],[+1,0],[0,-1],[0,+1]]
    cross_neighbours = [[-1,-1],[+1,-1],[-1,+1],[+1,+1]]
    def __init__(self, map):
        self.map = map

        self.width = len(self.map[len(self.map)-1])
        self.height = len(self.map)

    def is_limit(self, nx, ny):
        return ny > self.height - 1 or ny < 0 or nx > self.width -1 or nx < 0

    def get_neighbour(self, current):
        x = current.x
        y = current.y
        ngh_nodes = []

        for node_idx in self.neighbours:

            nx = x+node_idx[0]
            ny = y+node_idx[1]

            if self.is_limit(nx, ny):
                continue                
            if self.map[ny][nx] ==0:
                ngh_nodes.append(Node(nx,ny))
        for node_idx in self.cross_neighbours:
            nx = x+node_idx[0]
            ny = y+node_idx[1]

            if self.is_limit(nx, ny):
                continue

            if self.map[ny][nx] == 0 and self.map[y][nx] == 0 and self.map[ny][x] == 0:
                ngh_nodes.append(Node(nx,ny))

        return ngh_nodes

    def search(self, start, goal, h):
        
        previous_node = {}
        gScore = {}
        gScore[start] = 0
        fScore = {}
        fScore[start] = h(start, goal)

        open_list = [start]
        closed_list = []

        while len(open_list)>0:

            current = open_list[0]
            current_index=0
            for index, item in enumerate(open_list):
                if fScore[item] < fScore[current]:
                    current = item
                    current_index = index

            if current ==goal:
                return return_path(previous_node, current)

            open_list.pop(current_index)
            closed_list.append(current)

            for neighbor in self.get_neighbour(current):

                if neighbor in closed_list:
                    continue

                g_score_n = gScore[current] + 1

                if neighbor in open_list:
                   if g_score_n > gScore[neighbor]:
                       continue

                gScore[neighbor] = g_score_n
                fScore[neighbor] = g_score_n + h(neighbor, goal)
                previous_node[neighbor] = current
                open_list.append(neighbor)

        return None



def main(args):
  
    rospy.init_node('astar', anonymous=True)

    evade_obj = AStar()

    rospy.spin()



if __name__=='__main__':
	main(sys.argv)