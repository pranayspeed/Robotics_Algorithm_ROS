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
        return abs(np.cross(self.end-self.start, self.start-pt)/np.linalg.norm(self.end-self.start))

    
    def get_alignment_angle(self, other_line):
        vec1 = other_line.end - other_line.start
        vec1 = vec1/np.linalg.norm(vec1)
        vec2 = self.end - self.start
        vec2 = vec2/np.linalg.norm(vec2)
        return  math.degrees(math.atan2(vec2[1],vec2[0]) - math.atan2(vec1[1],vec1[0]))
    

    def get_closest_point_from_pt(self, pt):

        vec = self.end-self.start
        uvec = vec / np.linalg.norm(vec)
        pt_vec = pt - self.start
        dist_proj = np.dot(pt_vec, uvec)
        proj_vector = uvec * dist_proj
        pt_on_line = self.start + proj_vector

        return pt_on_line



    def get_ros_points(self):
        return [Point(self.start[0],self.start[1],0.0),Point(self.end[0],self.end[1],0.0)]

    
    def get_rotated_vector(self, degree_angle):
        rad_angle = math.radians(degree_angle)
        x = self.end[0] * math.cos(rad_angle) - self.end[0] * math.sin(rad_angle)
        y = self.end[1] * math.sin(rad_angle) + self.end[1] * math.cos(rad_angle)

        return Line(np.array([0,0]), np.array([x,y]))