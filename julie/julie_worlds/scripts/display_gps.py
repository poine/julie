#!/usr/bin/env python


import os, logging, math, numpy as np, pyproj
import rospy, rospkg, sensor_msgs.msg, visualization_msgs.msg, geometry_msgs.msg,  nav_msgs.msg

import test_03_make_rosmap as rm

class Node:
    def __init__(self, ref_name):
        rospack = rospkg.RosPack()
        jwd = rospack.get_path('julie_worlds')
        ref_filename = os.path.join(jwd, 'config/ref_{}.yaml'.format(ros_map_name))
        self.rf = rm.RosFrame(ref_filename)
        self.marker_gps_pub = rospy.Publisher('/display_gps/gps_marker', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber('/fix', sensor_msgs.msg.NavSatFix, self.navsat_cbk)
        rospy.Subscriber('/julie_gazebo/base_link_truth', nav_msgs.msg.Odometry, self.truth_cbk)
        
    def navsat_cbk(self, msg):
        self.loc_lla = [msg.longitude, msg.latitude, msg.altitude]
        self.loc_ros =  self.rf.world_to_ros(self.loc_lla)
        #print 'nav', self.loc_lla , self.loc_ros
        self.publish()
        
    def truth_cbk(self, msg):
        p = msg.pose.pose.position
        self.truth_ros = [p.x, p.y, p.z]
        #print 'truth', self.truth_ros, '->', self.rf.ros_to_world(self.truth_ros)
     
    def publish(self):
        msg = geometry_msgs.msg.PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = self.loc_ros[0]
        msg.pose.pose.position.y = self.loc_ros[1]
        msg.pose.pose.position.z = self.loc_ros[2]
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = 0, 0, 0, 1#tf.transformations.quaternion_from_euler(*[0, 0, self.start[2]])
        self.marker_gps_pub.publish(msg)
        
    def run(self):
        #rospy.spin()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            #print("hello123")
            #self.publish()
            rate.sleep()
        
    
if __name__ == '__main__':
    rospy.init_node('display_gps')
    ref_name = 'enac_outdoor_south_east'
    Node(ref_name).run()
