import numpy as np
import math

from geometry_msgs.msg import Point
class Line:
    #initialize goal line with start and end point
    def __init__(self, start_pt, end_pt):
        self.start = start_pt
        self.end = end_pt

    def mag(self):
        return np.linalg.norm(self.end-self.start)

    def get_shortest_dist_from_pt(self, pt):
        # p1 = self.start
        # p2 = self.end

        # #distance between p1 and p2
        # l2 = np.sum((p1-p2)**2)

        # #if you need the point to project on line extention connecting p1 and p2
        # t = np.sum((pt - p1) * (p2 - p1)) / l2

        # #if you need the point to project on line segment between p1 and p2 or closest point of the line segment
        # #t = max(0, min(1, np.sum((pt - p1) * (p2 - p1)) / l2))
        # t= np.sum((pt - p1) * (p2 - p1)) / l2
        # #print(t)
        # return t        
        return abs(np.cross(self.end-self.start, self.start-pt)/np.linalg.norm(self.end-self.start))

    
    def get_alignment_angle(self, other_line):
        vec1 = other_line.end - other_line.start
        vec1 = vec1/np.linalg.norm(vec1)
        vec2 = self.end - self.start
        vec2 = vec2/np.linalg.norm(vec2)
        return math.degrees(math.atan2(vec2[1],vec2[0]) - math.atan2(vec1[1],vec1[0]))
        

    def get_angle_of_normal_at_orgin(self):
        
        pt = np.array([0.0,0.0])
        
        x_axis = Line(pt, np.array([1.0,0.0]))

        normal_vec_line  = self.get_normal_from_pt(pt)
        normal_angle = normal_vec_line.get_alignment_angle(x_axis)
        print(normal_angle, "normal angle")
        return normal_angle
        #dist = np.linalg.norm(np.array(pt) - pt_on_line)
        #print("goal line distance", dist)

    def get_closest_point_from_pt(self, pt):
        # r1 = math.sqrt( ((self.start[0]- pt[0])**2)+((self.start[1]-pt[1])**2) )
        # r2 = math.sqrt( ((self.end[0]- pt[0])**2)+((self.end[1]-pt[1])**2) )
        # l = math.sqrt( ((self.start[0]- self.end[0])**2)+((self.start[1]-self.end[1])**2) )
        # x1 = (r1**2-r2**2 - l**2)/(2*l)
        # pt_on_line = self.start - (self.end - self.start)*x1 

        vec = self.end-self.start
        uvec = vec / np.linalg.norm(vec)
        pt_vec = pt - self.start
        dist_proj = np.dot(pt_vec, uvec)
        proj_vector = uvec * dist_proj
        pt_on_line = self.start + proj_vector

        #print(np.linalg.norm(pt_on_line1-pt_on_line)," diff in distance")

        return pt_on_line

        # p1 = self.start
        # p2 = self.end

        # #distance between p1 and p2
        # l2 = np.sum((p1-p2)**2)

        # #if you need the point to project on line extention connecting p1 and p2
        # t = np.sum((pt - p1) * (p2 - p1)) / l2

        # #if you need the point to project on line segment between p1 and p2 or closest point of the line segment
        # #t = max(0, min(1, np.sum((pt - p1) * (p2 - p1)) / l2))
        # t= np.sum((pt - p1) * (p2 - p1)) / l2

        # projection = p1 + t * (p2 - p1)
        
        # return projection




    def get_normal_from_pt(self, pt):
   
        pt_on_line = self.get_closest_point_from_pt(pt)
        
        return Line(pt, pt_on_line)

    def get_ros_points(self):
        return [Point(self.start[0],self.start[1],0.0),Point(self.end[0],self.end[1],0.0)]

    
    def get_rotated_vector(self, degree_angle):
        rad_angle = math.radians(degree_angle)
        x = self.end[0] * math.cos(rad_angle) - self.end[0] * math.sin(rad_angle)
        y = self.end[1] * math.sin(rad_angle) + self.end[1] * math.cos(rad_angle)

        #x=math.cos(rad_angle)*(self.end[0]-self.start[0])-math.sin(rad_angle)*(self.end[1]-self.start[1])   + self.start[0]
        #y=math.sin(rad_angle)*(self.end[0]-self.start[1])+math.cos(rad_angle)*(self.end[1]-self.start[1])  + self.start[1]

        return Line(np.array([0,0]), np.array([x,y]))