#!/usr/bin/env python

'''
   This is a quick rewrite of rbcar gazebo ackermann controller
'''

import math, time, numpy as np
import rospy
import ackermann_msgs.msg, gazebo_msgs.msg, std_msgs.msg, geometry_msgs, nav_msgs.msg, julie_navigation.msg
import tf.transformations

import pdb

def list_of_position(p): return (p.x, p.y, p.z)
def list_of_orientation(q): return (q.x, q.y, q.z, q.w)
# def normalize_pi02(a):
#     for i in range(len(a)):
#         while a[i]>math.pi: a[i] -= 2*math.pi
#         while a[i]<-math.pi: a[i] += 2*math.pi
        
''' Subscribe to gazebo/model_states messages and store the robot pose and twist '''
class ThruthListener:
    def __init__(self):
        self.car_model_name, self.car_model_idx = "julie", None
        rospy.Subscriber('/gazebo/model_states', gazebo_msgs.msg.ModelStates, self.callback)
        self.pose, self.twist = None, None 
        
    def callback(self, msg):
        if self.car_model_idx is None:
            try:
                self.car_model_idx = msg.name.index(self.car_model_name)
            except:
                rospy.logerr('model {} not found in gazebo {}'.format(self.car_model_name, msg.name))
        if self.car_model_idx is not None:
            self.pose, self.twist = msg.pose[self.car_model_idx], msg.twist[self.car_model_idx]
            self.vel = np.linalg.norm([self.twist.linear.x, self.twist.linear.y])
            self.om = self.twist.angular.z
            

''' Publish odometry over tf and as message '''
class OdometryPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/rbcar_robot_control/odom', nav_msgs.msg.Odometry, queue_size=1)
        self.br = tf.TransformBroadcaster()
    
    def publish(self, pose, twist):
        current_time = rospy.Time.now()
        # Publish the transform over tf
        self.br.sendTransform(list_of_position(pose.position), list_of_orientation(pose.orientation),
                              current_time, "base_footprint", "odom")
        # Publish odometry message
        msg = nav_msgs.msg.Odometry()
        msg.header.stamp, msg.header.frame_id = current_time, "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose = pose
        msg.pose.covariance = np.diag(0.1*np.ones(6)).reshape(36)
        msg.twist.twist = twist
        msg.twist.covariance =  np.diag(0.1*np.ones(6)).reshape(36)
        self.pub.publish(msg)

''' receive ackermann commands and control robot joints '''
class AckermannController:
    def __init__(self):
        # Car geometry. See https://docs.google.com/document/d/1SWTQJubQBMlC9y7mihsEORYDIJ67RsRUDWzbhw-RRqM/edit?usp=sharing
        self.wheel_r   = 0.47/2            # wheel radius
        self.wheel_l   = 1.65              # wheel longitudinal separation
        self.wheel_d   = 0.9               # lateral separation between rear wheels
        self.wheel_do2 = self.wheel_d/2    # half lateral separation between rear wheels
        self.wheel_d1 = 0.66               # lateral separation between front wheels pivots
        self.wheel_d1o2 = self.wheel_d1/2  # half lateral separation between front wheels pivots
        self.wheel_d2 = 0.12               # sepration between front wheel pivot and center of tire
        
        rospy.Subscriber('/julie_gazebo_ackermann_controller/command', ackermann_msgs.msg.AckermannDriveStamped, self.callback)
        self.last_command_time = None
        self.desired_steering, self.desired_vel = [0, 0, 0], [0, 0]

        self.wheels = ['right_front', 'left_front', 'right_rear', 'left_rear']
        self.wheel_vel_topics = ['/julie/{}_axle_controller/command'.format(wheel) for wheel in self.wheels]
        self.wheel_vel_pubs = [rospy.Publisher(topic, std_msgs.msg.Float64, queue_size=50) for topic in self.wheel_vel_topics]

        self.wheel_angle_topics = ['/julie/{}_steering_joint_controller/command'.format(side) for side in ['right', 'left']]
        self.wheel_angle_pubs = [rospy.Publisher(topic, std_msgs.msg.Float64, queue_size=50) for topic in self.wheel_angle_topics]
        
        
    def callback(self, msg):
        self.desired_steering = [msg.drive.steering_angle, msg.drive.steering_angle_velocity]
        self.desired_vel = [msg.drive.speed, msg.drive.acceleration, msg.drive.jerk]
        self.last_command_time = rospy.Time.now()


    def publish_joints_command(self):
        alpha = self.desired_steering[0]
        if alpha == 0:
            wheels_angles = [0, 0]
            wheels_rvel = -self.desired_vel[0]/self.wheel_r*np.ones(4)
            R = float('inf')
        else:
            R = self.wheel_l / math.tan(alpha) # turn radius
            s = np.sign(alpha)
            wheels_angles = [math.atan(self.wheel_l/(R+self.wheel_d1o2)), math.atan(self.wheel_l/(R-self.wheel_d1o2))]
            #print 'alpha {:.2f} v {:.2f} R {:.2f} wheels angles (r/l) {:.3f}/{:.3f}'.format(alpha, self.desired_vel[0], R, *wheels_angles)
            om = self.desired_vel[0]/R         # turn rate
            wheels_rvel = -self.desired_vel[0]/self.wheel_r*np.ones(4)#np.zeros(4)#
            # rear wheels
            wheels_rvel[2:] = -np.array([R+self.wheel_do2, R-self.wheel_do2])*om/self.wheel_r
            #front wheels
            foo = np.array([(R+self.wheel_d1o2)**2+self.wheel_l**2, (R-self.wheel_d1o2)**2+self.wheel_l**2])
            wheels_rvel[:2] = -np.sign(alpha)*(np.sqrt(foo)+[self.wheel_d2, -self.wheel_d2])*om/self.wheel_r
            #print wheels_rvel[:2]
            #print wheels_rvel[2:]
            print 'alpha {:.2f} v {:.2f} R {:.2f} wheels vel {}'.format(alpha, self.desired_vel[0], R, wheels_rvel)

        for wheel_rvel, pub in zip(wheels_rvel, self.wheel_vel_pubs):
            msg = std_msgs.msg.Float64(); msg.data = wheel_rvel
            pub.publish(msg)

        for wheel_angle, pub in zip(wheels_angles, self.wheel_angle_pubs):
            msg = std_msgs.msg.Float64(); msg.data = wheel_angle
            pub.publish(msg)

    
    def check_command_timeout(self):
        if self.last_command_time is None or (rospy.Time.now()-self.last_command_time).to_sec() > 0.1:
            self.desired_steering, self.desired_vel = [0, 0, 0], [0, 0]



class Node:

    def run(self, freq=50):
        rospy.init_node('julie_gazebo_ackermann_controller', anonymous=True)
        self.truth = ThruthListener()
        self.op = OdometryPublisher()
        self.ac = AckermannController()
        self.debug_pub = rospy.Publisher('/rbcar_robot_control/debug', julie_navigation.msg.debug_ackermann_controller, queue_size=1)
        
        rate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            if self.truth.pose is not None:
                self.op.publish(self.truth.pose, self.truth.twist)
                self.publish_debug()
            self.ac.check_command_timeout()
            self.ac.publish_joints_command()
            rate.sleep()
            
    def publish_debug(self):
        alpha = np.arctan2(self.ac.wheel_l*self.truth.om, self.truth.vel)
        #print('v {:.2f}/{:.2f} a {:.2f}/{:.2f}'.format(self.truth.vel, self.ac.desired_vel[0], alpha, self.ac.desired_steering[0]))
        msg = julie_navigation.msg.debug_ackermann_controller()
        msg.vel_sp, msg.vel = self.ac.desired_vel[0], self.truth.vel
        msg.steering_sp, msg.steering = self.ac.desired_steering[0], alpha
        self.debug_pub.publish(msg)

if __name__ == '__main__':
    try:
        Node().run()
    except rospy.ROSInterruptException:
        pass
