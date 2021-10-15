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

from Line import Line



class RANSAC:
    """ Implement ransac algo to display lines
    """
    def __init__(self):
        
        scan_topic = rospy.get_param("~scan_topic")
        self.base_link_topic = rospy.get_param("~base_link_topic")

        self.threshold_for_triggering_ransac = rospy.get_param("~threshold_for_triggering_ransac")
        self.inliner_threshold = rospy.get_param("~inliner_threshold")
        self.ransac_iteration_count = rospy.get_param("~ransac_iteration_count")


        ransac_lines_topic = rospy.get_param("~ransac_lines")

        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.check_fitting)

        self.marker_pub = rospy.Publisher(ransac_lines_topic, Marker, queue_size=10)
  


    def check_fitting(self,msg):
        self.fit_lines(msg)


    def publish_lines(self, lines):
        if len(lines)>0: # check for list is not empty
            fitted_lines = Marker()
            fitted_lines.type = Marker.LINE_LIST
            fitted_lines.scale.x=0.2
            fitted_lines.scale.y=0.2
            fitted_lines.color = ColorRGBA(1.0,1.0,0,1)   
            fitted_lines.header.frame_id= self.base_link_topic     
            for line in lines:
                [p1,p2] = line.get_ros_points()
                fitted_lines.points.append(p1)
                fitted_lines.points.append(p2)
            self.marker_pub.publish(fitted_lines)

    def fit_lines(self, msg):

        points = Marker()
        points.type = Marker.POINTS
        points.scale.x=0.1
        points.scale.y=0.1
        points.color = ColorRGBA(1.0,0,0,1)

        points.header.frame_id= self.base_link_topic

        angle_min =        msg.angle_min
        angle_increment =  msg.angle_increment

        if len(msg.intensities)>0:
            filtered_index = [idx for idx, val in enumerate(msg.intensities) if val >0.5]
            if len(filtered_index)<2:
                filtered_index =  [idx for idx, val in enumerate(msg.ranges)]
        else:
            filtered_index =  [idx for idx, val in enumerate(msg.ranges)]
        
        for pt_indx in filtered_index: #range(len(msg.ranges)):

            pt_angle = angle_min + pt_indx*angle_increment

            x = msg.ranges[pt_indx]* math.cos(pt_angle)
            y = msg.ranges[pt_indx]* math.sin(pt_angle)
            points.points.append(Point(x,y,0.0))
        
        lines = self.ransac(points.points)
        self.publish_lines(lines)



    def ransac(self, points):
        
        #choose number of points, for line need only two points
        n_pt=2
        #choose point threshold in percent, for termination condition
        threshold_per = 0.20 # 5% remaining

        #inliner threshold
        inliner_threshold_dist = self.inliner_threshold

        # no. of iteration
        k=self.ransac_iteration_count

        #create a copy and process, since the process needs elemination
        point_list=points
        
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
                while p1 == p2: #make sure that different points are selected
                    p1 = random.choice(point_list)
                    p2 = random.choice(point_list) 

                selected_line = Line(np.array([p1.x,p1.y]),np.array([p2.x,p2.y]))                  

                #Draw a vector
                inliner_count = 0
                out_liners = []
 
                #loop points
                for pt in point_list:
                    #Find distance from the point

                    pt_dist=np.abs(selected_line.get_shortest_dist_from_pt(np.array([pt.x,pt.y])))
                    if  pt_dist < inliner_threshold_dist:
                        inliner_count+=1
                    else:
                        out_liners.append(pt)

                if inliner_count > max_inliers:
                    max_inliers = inliner_count

                    best_line = selected_line
            if best_line:
                lines.append(best_line)
                
            point_list = out_liners

        return lines




def main(args):
  
  rospy.init_node('perception', anonymous=True)
  evade_obj = RANSAC()
  rospy.spin()



if __name__=='__main__':
	main(sys.argv)