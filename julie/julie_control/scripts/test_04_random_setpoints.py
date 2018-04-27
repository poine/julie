#!/usr/bin/env python
import os, sys , math, numpy as np, rospy, ackermann_msgs.msg, nav_msgs.msg, tf.transformations, geometry_msgs.msg
import roslib, rospkg
import julie_misc.utils as jmu


def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1

class Node:
    def __init__(self):
        self.alpha, self.vel = 0., 3.

        self.phase, self.xlim, self.vlim = 0, 10, 3.
        
        ackermann_cmd_topic = '/julie/julie_ackermann_controller/cmd_ack'
        self.pub_ackermann = rospy.Publisher(ackermann_cmd_topic, ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('/julie_gazebo/base_link_truth', nav_msgs.msg.Odometry, self.odom_cbk)

    def odom_cbk(self, msg):
        self.pose = msg.pose.pose
        self.twist = msg.twist.twist
        
    def publish_ackermann(self):
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.drive.steering_angle = self.alpha
        msg.drive.speed = self.vel
        self.pub_ackermann.publish(msg)

    def update_setpoint(self):
        #t = rospy.Time.now().to_sec()
        #self.vel = step(t)
        try:
            v = jmu.list_of_xyz(self.twist.linear)[0]
            x = jmu.list_of_xyz(self.pose.position)[0]
        except AttributeError:
            print 'no pose'
        else:
            if self.phase == 0:
                if x > self.xlim:
                    self.vel = -self.vlim
                    self.phase = 1
            else:
                if x < -self.xlim:
                    self.vel = self.vlim
                    self.phase = 0 
            print('{} {}: {}'.format(x, v, self.vel))
            
            
    def run(self):
        self.rate = rospy.Rate(50.)
        while not rospy.is_shutdown():
            self.update_setpoint()
            self.publish_ackermann()
            self.rate.sleep()

            
        
def main(args):
    rospy.init_node('julie_control__test_04_random_setpoint')
    Node().run()


if __name__ == '__main__':
    main(sys.argv)
