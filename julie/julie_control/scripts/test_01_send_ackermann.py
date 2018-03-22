#!/usr/bin/env python
import roslib
import sys , math, numpy as np, rospy, ackermann_msgs.msg


class Node:
    def __init__(self, vel=0., alpha=0.):
        self.vel, self.alpha = vel, alpha
        ackermann_cmd_topic = '/julie_gazebo_ackermann_controller/command'
        self.pub = rospy.Publisher(ackermann_cmd_topic, ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)

    def publish_ack(self):
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.drive.steering_angle = self.alpha
        msg.drive.speed = self.vel
        self.pub.publish(msg)
        
    def run(self):
        self.rate = rospy.Rate(50.)
        while not rospy.is_shutdown():
            self.publish_ack()
            self.rate.sleep()

def main(args):
    rospy.init_node('julie_control__test_01_send_ackermann')
    Node(vel=0.1, alpha=0.).run()
  

if __name__ == '__main__':
    main(sys.argv)
