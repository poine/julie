#!/usr/bin/env python
import os, sys , math, numpy as np, rospy, ackermann_msgs.msg, nav_msgs.msg, tf.transformations, geometry_msgs.msg
import roslib, rospkg

import julie_control.two_d_guidance as tdg

import pdb

def list_of_xyz(p): return [p.x, p.y, p.z]
def array_of_xyz(p): return np.array(list_of_xyz(p))
def list_of_xyzw(q): return [q.x, q.y, q.z, q.w]

class Node:
    def __init__(self, path_filename):
        param = tdg.pure_pursuit.Param()
        param.L = 2
        self.ctl = tdg.pure_pursuit.PurePursuit(path_filename, param, look_ahead=5.5)
        self.vel = 3.
        
        ackermann_cmd_topic = '/julie_gazebo_ackermann_controller/command'
        self.pub_ackermann = rospy.Publisher(ackermann_cmd_topic, ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)
        self.pub_path = rospy.Publisher('pure_pursuit/path', nav_msgs.msg.Path, queue_size=1)
        
        rospy.Subscriber('/julie_gazebo/base_link_truth', nav_msgs.msg.Odometry, self.odom_cbk)

    def odom_cbk(self, msg):
        self.pose = msg.pose.pose
        
    def publish_ackermann(self):
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.drive.steering_angle = self.alpha
        msg.drive.speed = self.vel
        self.pub_ackermann.publish(msg)

    
    def publish_path(self, _path):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="map"
        for l in _path.points:
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x, pose.pose.position.y = l
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)

        
    def run(self):
        self.rate = rospy.Rate(50.)
        while not rospy.is_shutdown():
            try:
                l = array_of_xyz(self.pose.position)[:2]
                y = tf.transformations.euler_from_quaternion(list_of_xyzw(self.pose.orientation))[2]
            except AttributeError:
                print 'no pose'
            else:
                try:
                    _unused, self.alpha = self.ctl.compute(l, y)
                except tdg.pure_pursuit.EndOfPathException:
                    self.ctl.path.reset()
                    _unused, self.alpha = self.ctl.compute(l, y)
                else:
                    self.publish_ackermann()
            self.publish_path(self.ctl.path)
            self.rate.sleep()

        

def make_oval(filename='/tmp/foo'):
    c1, c2, r = np.array([17.5, -35.]), np.array([17.5, 13.]), 7.
    path = tdg.path_factory.make_oval_path(c1, c2, r)
    path.save(filename)

def main(args):
    #make_oval()
    rospy.init_node('julie_control__test_02_pure_pursuit')
    jwd = rospkg.RosPack().get_path('julie_worlds')
    path_filename = os.path.join(jwd, 'paths/enac_outdoor_south_east/path_J_1.npz')
    Node(path_filename).run()
  

if __name__ == '__main__':
    main(sys.argv)
