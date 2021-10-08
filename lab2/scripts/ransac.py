#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist, Point, Vector3


from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA



import math

import numpy as np
import random


class RANSAC:
    """ Implement ransac algo to display lines
    """
    def __init__(self):
        

        scan_topic = rospy.get_param("~scan_topic")
        drive_topic = rospy.get_param("~goal_seeker_drive_topic")
        self.threshold_for_triggering_ransac = rospy.get_param("~threshold_for_triggering_ransac")
        self.inliner_threshold = rospy.get_param("~inliner_threshold")
        self.ransac_iteration_count = rospy.get_param("~ransac_iteration_count")


        gt_pose_topic = rospy.get_param("~gt_pose_topic")


        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.check_fitting)
        self.drive_pub = rospy.Publisher(drive_topic, Twist,queue_size=10)

        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
  
        
    def seek_goal(self, msg):
        twist_msg = Twist()
        twist_msg.linear= Vector3(1,0,0)
        self.drive_pub.publish(twist_msg)


    def check_fitting(self,msg):
        min_range = min(msg.ranges)
        if min_range <self.threshold_for_triggering_ransac:
            
            self.fit_lines(msg)
        else:
            print("ransac not triggered",min_range)

    def fit_lines(self, msg):

        list_of_points = msg.ranges

        points = Marker()
        points.type = Marker.POINTS
        points.scale.x=0.1
        points.scale.y=0.1
        points.color = ColorRGBA(1.0,0,0,1)

        points.header.frame_id= "odom"

        angle_min =        msg.angle_min
        angle_increment =  msg.angle_increment

        base_x=0
        base_y=0

        min_dist = 0.0
        for pt_indx in range(len(msg.ranges)):
            pt_angle = angle_min + pt_indx*angle_increment

            #print(pt_angle)
            x = msg.ranges[pt_indx]* math.cos(pt_angle)
            y = msg.ranges[pt_indx]* math.sin(pt_angle)
            points.points.append(Point(x,y,0.0))

            if pt_indx ==1:
                min_dist = np.linalg.norm(np.array([base_x,base_y]) - np.array([x,y]))

            base_x=x
            base_y=y


        


        

        lines = self.ransac(points.points, min_dist)

        if len(lines)>0:
            lines_marker = Marker()
            lines_marker.type = Marker.LINE_LIST
            lines_marker.scale.x=0.2
            lines_marker.scale.y=0.2
            lines_marker.color = ColorRGBA(1.0,1.0,0,1)

            lines_marker.header.frame_id= "odom"

            for line in lines:
                lines_marker.points.append(line[0])
                lines_marker.points.append(line[1])
            self.marker_pub.publish(lines_marker)
            #print(lines,"-----------------", len(lines))
        #self.marker_pub.publish(points)

    def ransac(self, points,min_dist):
        
        #choose number of points, for line need only two points
        n_pt=2
        #choose point threshold in percent, for termination condition
        threshold_per = 0.30

        #inliner threshold
        inliner_threshold_dist = self.inliner_threshold


        # no. of iteration
        k=self.ransac_iteration_count

        #create a copy and process, since the process needs elemination
        point_list=points#.copy()
        
        termination_point_count = int (len(point_list)*threshold_per)

        #return list of lines
        lines = []

        
        #terminate if points are less that threshold
        while len(point_list) > termination_point_count:
            # best fitted line points to be added to list
            best_line = None
            max_inliers=0            
            out_liners = None
            for i in range(k):

                #Pick 2 points at random
                
                p1 = random.choice(point_list)
                p2 = random.choice(point_list)
                
                p1
                #p1 = rand_pts[0]
                #p2 = rand_pts[1]
                #rand_pts = [[p1.x,p1.y],[p2.x,p2.y]]
                #Draw a vector
                inliner_count = 0
                out_liners = []
                
                #loop points
                for pt in point_list:
                    #Take dot product for each point
                    pts1 = np.array([p1.x,p1.y])
                    pts2 = np.array([p2.x,p2.y])
                    pts3 = np.array([pt.x,pt.y])
                    pt_dist=0.0
                    try:
                        pt_dist = np.abs(np.linalg.norm(np.cross(pts2-pts1, pts1-pts3)))/np.linalg.norm(pts2-pts1)
                    except:
                        pass

                    #pt_dist = np.dot(np.array([p1.x,p1.y])-np.array([p2.x,p2.y]), np.array([pt.x,pt.y]))
                    if  pt_dist < inliner_threshold_dist:
                        inliner_count+=1
                    else:
                        out_liners.append(pt)
                #selected wrong points
                if inliner_count<termination_point_count:
                    continue

                if inliner_count > max_inliers:
                    max_inliers = inliner_count

                    best_line = [p1,p2]
            if best_line:
                lines.append(best_line)
                if len(lines)>3:
                    break
                
            point_list = out_liners

        return lines




def main(args):
  
  rospy.init_node('ransac', anonymous=True)
  evade_obj = RANSAC()
  rospy.spin()



if __name__=='__main__':
	main(sys.argv)