#!/usr/bin/env python
import sys
import rospy

from geometry_msgs.msg import PoseStamped, TransformStamped

import tf2_ros


class Pursuer:
    """ Implement Wall Following on the car
    """
    def __init__(self, pursuername, evadername):

        self.base_frame = rospy.get_param("~base_frame")
        self.map_frame = rospy.get_param("~map_frame")
        self.pursuername = pursuername

        gt_pose_topic = rospy.get_param("~ground_truth_pose_topic")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        gt_pose_topic = rospy.get_param("~ground_truth_pose_topic")
        self.pose_sub = rospy.Subscriber(gt_pose_topic , PoseStamped, self.persue_walk)

        self.evadername = evadername

 

    def persue_walk(self, drive_msg):
        # Send Transform to pursuer

        when = 1.0  # Follow by 1 second
        past = rospy.Time.now() - rospy.Duration(when)
        source_frame=self.evadername+"/"+self.base_frame
        target_frame=self.map_frame

        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame  
        t.child_frame_id = self.pursuername+"/"+self.base_frame
        try:

          if self.tfBuffer.can_transform(target_frame, source_frame, past):
            trans = self.tfBuffer.lookup_transform(target_frame, source_frame, past,rospy.Duration(0.01))
            t.transform = trans.transform

            br.sendTransform(t)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          print(sys.exc_info()[1]) 

        try:
            #Steer angle 
            for component in ["left", "right"]:
              source_frame=self.evadername+"/front_"+component+"_wheel"
              target_frame=self.evadername+"/front_"+component+"_hinge"
              if self.tfBuffer.can_transform(target_frame, source_frame, past):
                trans = self.tfBuffer.lookup_transform(target_frame, source_frame, past)

                br = tf2_ros.TransformBroadcaster()
                t = TransformStamped()

                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.pursuername+"/front_"+component+"_hinge"
                t.child_frame_id = self.pursuername+"/front_"+component+"_wheel"
                t.transform = trans.transform

                br.sendTransform(t)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print(sys.exc_info()[1])   


def main(args):
  
  rospy.init_node('pursuer', anonymous=True)
  evadername=rospy.get_param("~racecar_frame")
  pursuername=rospy.get_param("~pursuer_frame")
  prusue_obj = Pursuer(pursuername,evadername)
  rospy.spin()



if __name__=='__main__':
	main(sys.argv)
    

