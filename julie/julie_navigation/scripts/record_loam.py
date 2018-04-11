#!/usr/bin/env python


import os, logging, math, numpy as np
import rospy, rospkg, sensor_msgs.msg, nav_msgs.msg

import julie_worlds
import pdb

''' 
  Record GPS (in LTP) and Loam during run
'''


class Node:
    def __init__(self, ref_name):
        rospy.init_node('record_loam')
        jwd = rospkg.RosPack().get_path('julie_worlds')
        ref_filename = os.path.join(jwd, 'config/ref_{}.yaml'.format(ref_name))
        self.rf = julie_worlds.LTPFrame(ref_filename)
        self.gps_loc, self.loam_loc = [], []
        rospy.Subscriber('/integrated_to_init', nav_msgs.msg.Odometry, self.loam_cbk)
        rospy.Subscriber('/ublox_gps/fix', sensor_msgs.msg.NavSatFix, self.navsat_cbk, 'pos')
        
    def navsat_cbk(self, msg, arg):
        loc_lla = [msg.longitude, msg.latitude, msg.altitude]
        p_cov = [msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8]]
        print p_cov
        self.gps_loc.append([msg.header.stamp.to_sec()]  + self.rf.world_to_ros(loc_lla))
        
    def loam_cbk(self, msg):
        p = msg.pose.pose.position
        self.loam_loc.append([msg.header.stamp.to_sec(), p.x, p.y, p.z])
            
    def run(self):
        #rospy.spin()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()


def record_and_save(ltp_filename, output_filename):
    node = Node(ltp_filename)
    try:
        node.run()
    except rospy.ROSInterruptException:
        print('recorded {} gps and {} lidar'.format( len(node.gps_loc), len(node.loam_loc)))
    print('saving to {}'.format(output_filename))
    np.savez(output_filename, gps=np.array(node.gps_loc), loam=np.array(node.loam_loc))


if __name__ == '__main__':
    np.set_printoptions(precision=6)
    record_and_save(ltp_filename='enac_outdoor_south_east', output_filename='gps_loam')
