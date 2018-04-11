#!/usr/bin/env python


import os, logging, math, numpy as np, pyproj
import rospy, rospkg, sensor_msgs.msg, visualization_msgs.msg, geometry_msgs.msg, nav_msgs.msg, geographic_msgs.msg

import julie_worlds
import pdb
''' 
  Converts sensor_msgs/NavSatFix to geographic_msgs/GeoPointStamped
  This was made to input GPS in HDL SLAM
'''


class Node:
    def __init__(self, ref_name):
        jwd = rospkg.RosPack().get_path('julie_worlds')
        ref_filename = os.path.join(jwd, 'config/ref_{}.yaml'.format(ref_name))
        self.geopoint_pub = rospy.Publisher('/gps/geopoint', geographic_msgs.msg.GeoPointStamped, queue_size=1)
        rospy.Subscriber('/ublox_gps/fix', sensor_msgs.msg.NavSatFix, self.navsat_cbk, 'pos')
        
    def navsat_cbk(self, msg, arg):
            self.msg_header = msg.header
            self.loc_lla = [msg.longitude, msg.latitude, msg.altitude]
            self.publish()

    def publish(self):
        msg = geographic_msgs.msg.GeoPointStamped()
        msg.header = self.msg_header
        msg.position.latitude  = self.loc_lla[1]
        msg.position.longitude = self.loc_lla[0]
        msg.position.altitude  = self.loc_lla[2]
        self.geopoint_pub.publish(msg)
        
    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
        
    
if __name__ == '__main__':
    np.set_printoptions(precision=6)
    rospy.init_node('convert_gps')
    ref_name = 'enac_outdoor_south_east'
    Node(ref_name).run()
