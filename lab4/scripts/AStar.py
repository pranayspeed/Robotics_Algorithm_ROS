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

from sensor_msgs.msg import LaserScan
from itertools import groupby
from operator import itemgetter

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


class Bug2:
    mode_list = ["GOAL_SEEK","WALL_FOLLOW","GOAL_REACHED"]
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
        self.motion_state = self.motion_list[0]

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

        self.last_orientation =None

        self.front_collision_angle_range=10

        
    def get_goal_distance(self):
        return self.car_current_state.get_distance_from_point(self.goal_pt)


    def get_obstacle_angle(self, msg):

        angles_list = self.get_collision_limit_angles(msg)
        if angles_list is None:
            return None
        
        return math.degrees(angles_list[0][0])


    def follow_wall(self):
        self.mode = self.mode_list[1] # wall follow state
        self.motion_state = self.motion_list[0] # need to rotate to allign parallel to wall

        self.required_rotation = self.rotation_angle
        self.last_change_pt = self.car_current_state.get_position_vector()

    def follow_goal(self):
        self.mode = self.mode_list[0] # goal follow state
        self.motion_state = self.motion_list[1] # need to rotate to allign parallel to goal line

        #get rotation angle to align with the goal line
        current_direction = self.car_current_state.get_direction_vector()
        self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
                
        self.last_change_pt = None 


    def goal_seek(self, msg):
               
        linear_motion = [0.5,0,0]
        angular_motion =[0,0,0]

        current_direction = self.car_current_state.get_direction_vector()

        goal_dist = self.get_goal_distance()
        if goal_dist <0.1:
            print("goal Reached..........")
            linear_motion = [0,0,0]
            self.required_rotation = 0
            self.mode = self.mode_list[2]

        elif goal_dist < 0.5:
            print("goal reaching..........")

            self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
            kp=0.5
            ki=0 
            kd=0.01        
            linear_motion[0] = self.pid_control(goal_dist, kp,ki,kd) 


        else:
            
            if self.check_collosion_dist_at_angle(msg, 0, self.front_collision_angle_range) < 0.6: #detected wall                    
                self.follow_wall()
                linear_motion = [0,0,0]
            else:
                self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
                
                if math.fabs(self.required_rotation) < 0.1:
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
        augular_velocity = self.pid_control(self.required_rotation, kp,ki,kd)                
        
        
        angular_motion = [0,0,augular_velocity]

        return [linear_motion,angular_motion]

    def wall_follow(self, msg):
       
        wall_follow_velocity = 0.8

        d = self.goal_line.get_shortest_dist_from_pt(self.car_current_state.get_position_vector())
            
        ignore_point = False
        if self.last_change_pt is not None:
            dist_val = self.car_current_state.get_distance_from_point(self.last_change_pt)
            ignore_point = dist_val < 0.7
        if d < 0.08 and not ignore_point: #approaching goal line
            self.follow_goal()
            linear_motion = [0,0,0]
            angular_motion = [0,0,0]
            return [linear_motion,angular_motion]
        else:
            [pid_error, actual_velocity] = self.calculate_wall_error_velocity(msg,wall_follow_velocity)
            
            angular_velocity = self.pid_control(pid_error, self.kp,self.ki,self.kd)

            linear_motion = [actual_velocity,0,0]
            angular_motion =[0,0,angular_velocity]
            return [linear_motion,angular_motion]
     

    def calculate_wall_error_velocity(self, msg, velocity):
        #print("wall distance ",wall_dist, self.get_obstacle_angle(msg))
        obstacle = self.get_collision_limit_angles(msg)

        alpha=math.radians(30)
        if obstacle is not None:
            #take the leftmost collision limits
            ob_left = obstacle[0]

            angle = ob_left[0] - ob_left[1]
            if angle !=0:
                obstacle_index = self.get_collision_limit_index(msg)

                left_obstacle_index = obstacle_index[0]
                a = msg.ranges[left_obstacle_index[0]]
                b = msg.ranges[left_obstacle_index[1]]
                
                if a!=0:
                    alpha = math.atan(((a*math.cos(angle))-b)/(a*math.sin(angle)))
         
        else:
            alpha=math.radians(30)


        if self.check_collosion_dist_at_angle(msg, 0, self.front_collision_angle_range*1.5) < 0.5:
            error = -1
            velocity=0.0
        else:
            wall_dist = self.check_collosion_dist_at_angle(msg,80,10)
            d_1 = wall_dist + self.look_ahead*math.sin(alpha)            
            error = d_1 - self.wall_follow_dist

        return [error, velocity]     



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


    def get_collision_limit_index(self, msg):
        angle_index_list=[]
        filtered_index = [idx for idx, val in enumerate(msg.intensities) if val >0.5]
        if len(filtered_index)<1:
            return None
        else:
            for k, g in groupby(enumerate(filtered_index), lambda i_x: i_x[0] - i_x[1]):
                l1 = list(map(itemgetter(1), g))
                angle_index_list.append([l1[0], l1[-1]])

        return angle_index_list

    def get_collision_limit_angles(self, msg):

        angle_list=[]
        angle_index_list = self.get_collision_limit_index(msg)
        if angle_index_list is None:
            return None
        else:
            for l1 in angle_index_list:
                angle_list.append([msg.angle_min + l1[0]*msg.angle_increment,msg.angle_min + l1[-1]*msg.angle_increment])

        return angle_list



    def check_collosion_dist_at_angle(self, msg, angle ,angle_range=0):

        rad_angle = math.radians(angle)
        #half range for start angle
        rad_angle_range = math.radians(angle_range)/2

        start_angle =rad_angle - rad_angle_range

        dff_from_min = start_angle - msg.angle_min

        start_index = int(dff_from_min/msg.angle_increment)
        end_index = int(start_index + 2*(rad_angle_range/msg.angle_increment))
        if angle_range ==0:
            return msg.ranges[start_index]
        return min(msg.ranges[start_index: end_index])            

    def laser_move(self, msg):

        #Goal Seak
        if self.mode == self.mode_list[0]: #goal seeking
            [linear_motion,angular_motion] = self.goal_seek(msg)

        elif self.mode == self.mode_list[1]: #wall following
            [linear_motion,angular_motion] = self.wall_follow(msg)
        else:
            print("reached goal")
            linear_motion = [0,0,0]
            angular_motion =[0,0,0]

        return [linear_motion,angular_motion]

    def move(self, current_line):
               
        linear_motion = [0.2,0,0]
        angular_motion =[0,0,0]

        current_direction = self.car_current_state.get_direction_vector()

        self.required_rotation = current_line.get_alignment_angle(current_direction)
         
        goal_dist = self.get_goal_distance()
        if goal_dist <0.1:
            print("goal Reached..........")
            linear_motion = [0,0,0]
            self.required_rotation = 0
            self.mode = self.mode_list[2]

        elif goal_dist < 0.5:
            print("goal reaching..........")

            self.required_rotation = self.goal_line.get_alignment_angle(current_direction)
            kp=0.2
            ki=0 
            kd=0.01        
            linear_motion[0] = self.pid_control(goal_dist, kp,ki,kd) 


        else:                
            if math.fabs(self.required_rotation) < 0.1:
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
        augular_velocity = self.pid_control(self.required_rotation, kp,ki,kd)                
        
        
        angular_motion = [0,0,augular_velocity]

        return [linear_motion,angular_motion]



class AStar:
    """ Implement ransac algo to display lines
    """
    def __init__(self, path_list):

        self.path_list = path_list

        self.curr_index = 0
        #new Implementation
        drive_topic = rospy.get_param("~goal_seeker_drive_topic")
        odom_topic = rospy.get_param("~odom_topic")

        scan_topic = rospy.get_param("~scan_topic")

        start_pt = np.array(rospy.get_param("~start_pt"))
        goal_pt = np.array(rospy.get_param("~goal_pt"))

        self.drive_pub = rospy.Publisher(drive_topic, Twist,queue_size=10)

        car_state = CarState([start_pt[0],start_pt[1],90.0])
        

        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.update_pos)

        #self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.laser_move)


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

    def stop_move(self):
        if self.curr_index > len(self.path_list)-1:
            twist_msg = Twist()
            twist_msg.linear= Vector3(0,0,0)            
            twist_msg.angular= Vector3(0,0,0)    
            self.drive_pub.publish(twist_msg)
            return True
        return False

    def update_pos(self,msg):
        #Get odometry and execute the algorithm
        #update car current state
        self.motion_algo.car_current_state.update_pose(msg)
        

        ci = self.curr_index

        if self.stop_move():
            return

        if ci+1 != len(self.path_list):

            p1 = np.array(self.path_list[ci])
            p2 = np.array(self.path_list[ci+1])

            p1 = np.array([p1[0]+0.5,p1[1]-0.5])
            p2 = np.array([p2[0]+0.5,p2[1]-0.5])
        else:
            p1 = np.array(self.path_list[ci-1])
            p2 = np.array(self.path_list[ci])

            p1 = np.array([p1[0]+0.5,p1[1]-0.5])
            p2 = np.array([p2[0]+0.5,p2[1]-0.5])            

        current_line = Line(p1,p2)

        if self.motion_algo.car_current_state.is_ahead(current_line):

            print("Next Move")
            print(self.motion_algo.car_current_state.get_position_vector())
            self.curr_index+=1
            if self.stop_move():
                return
            ci = self.curr_index
            if ci+1 != len(self.path_list):
                p1 = np.array(self.path_list[ci])
                p2 = np.array(self.path_list[ci+1])
                p1 = np.array([p1[0]+0.5,p1[1]-0.5])
                p2 = np.array([p2[0]+0.5,p2[1]-0.5])
                current_line = Line(p1,p2)

        print(p1,p2)
        [linear, angular] = self.motion_algo.move(current_line)
        
        twist_msg = Twist()
        twist_msg.linear= Vector3(linear[0],0,0)
        rotation_val = angular[2]
        
        twist_msg.angular= Vector3(0,0,rotation_val)    

        self.drive_pub.publish(twist_msg)




class Node_n:
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
    def __init__(self, map):
        self.map = map

        self.width = len(map[0])
        self.height = len(map)

    def get_neighbour(self, current):
        x = current.x
        y = current.y
        ngh_nodes = []

        neighbour_list = [[x-1,y],[x+1,y],[x,y-1],[x,y+1]]
        for node_idx in neighbour_list:
            nx = node_idx[0]
            ny = node_idx[1]
            if ny > (len(self.map) - 1) or ny < 0 or nx > (len(self.map[len(self.map)-1]) -1) or nx < 0:
                continue
            #if nx >=0 and ny>=0 and nx < self.width and ny <self.height:
                
            if self.map[ny][nx] ==0:
                ngh_nodes.append(Node_n(nx,ny))
        return ngh_nodes

    def search(self, start, goal, h):
        
        cameFrom = {}
        gScore = {}
        gScore[start] = 0
        fScore = {}
        fScore[start] = h(start, goal)

        open_list = [start]
        closed_list = []

        while len(open_list)>0:
            # This operation can occur in O(1) time if openSet is a min-heap or a priority queue

            current = open_list[0]
            current_index=0
            for index, item in enumerate(open_list):
                if fScore[item] < fScore[current]:
                    current = item
                    current_index = index

            if current.x == goal.x and current.y == goal.y:
                return return_path(cameFrom, current)

            #open_list.remove(current)
            open_list.pop(current_index)
            closed_list.append(current)

            for neighbor in self.get_neighbour(current):

                if neighbor in closed_list:
                    continue

                tentative_gScore = gScore[current] + 1 #d(current, neighbor)
                g_score_n = tentative_gScore
                

                if neighbor in open_list:
                   if g_score_n > gScore[neighbor]:
                       continue

                gScore[neighbor] = g_score_n
                fScore[neighbor] = g_score_n + h(neighbor, goal)
                cameFrom[neighbor] = current
                open_list.append(neighbor)



        # Open set is empty but goal was never reached
        return None


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        #for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares
        for new_position in [(-1, 0), (1, 0), (0, -1), (0, 1)]: # Adjacent squares
            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


def main(args):
  
    rospy.init_node('astar', anonymous=True)
    #evade_obj = AStar()

    map_file = rospy.get_param('map_file')
    with open(map_file) as fd:
        map_str = fd.read()
        map_str = map_str.split('[')[1].split(']')[0]
        map_rows = map_str.split('\n')
        map = []
        #print(map_rows)

        for row in map_rows:
            map_current_row = []
            for val in row.split(','):
                val_str = val.strip()
                if len(val_str)>0:
                    map_current_row.append(int(val_str))
            map.append(map_current_row)

        xo = len(map[0])/2
        yo = len(map)/2

        print(map)


        sx = -8
        sy = -2
        gx = 4
        gy = 9

        # start = (xo + sx, yo - sy)
        # end = (xo + gx, yo - gy)

        # final_path = astar(map, start, end)

        astar_planner = AStarPlanner(map)
        start = Node_n(xo + sx, yo - sy)
        goal = Node_n(xo + gx, yo - gy) # 4.5 need to check if it should be 4.5

        final_path = astar_planner.search(start, goal, euclidean_dist)
        print(final_path)

        new_path = []

        for pt in final_path:
            new_path.append([ pt[0] - xo, - (pt[1] -yo)])
        print(new_path)
        evade_obj = AStar(new_path)

    rospy.spin()



if __name__=='__main__':
	main(sys.argv)