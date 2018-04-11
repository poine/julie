#!/usr/bin/env python


import os, logging, math, numpy as np, subprocess
import rospy, rospkg, sensor_msgs.msg, nav_msgs.msg
import tf.transformations

import julie_misc.three_d_plot as jmp, julie_misc.fit_trajectories as jmft
import julie_misc.algebra as jma, julie_misc.utils as jmu
import pdb

''' 
  Match GPS and Lidar trajectories

  TODO: remove the stupid eulers

'''
LOG = logging.getLogger('fit_loam')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def find_transform(filename, plot=False):
    LOG.info(' loading trajectories from {}'.format(filename))
    data =  np.load(filename)
    gps, loam = data['gps'], data['loam']

    t, l1, l2 = jmft.synchronyze(loam, gps)
    p = jmft.fit_horn(l1, l2)
    LOG.info(' computed transformation\n{}'.format(np.array(p)))
    l3 = jmft.transform(l1, p)

    
    if plot:
        margins, figsize =(0.03, 0.05, 0.98, 0.95, 0.06, 0.18), (0.75*20.48, 0.4*15.36)
        figure = jmp.prepare_fig(None, "Slam Run", figsize=figsize, margins=margins)

        ax = plt.gcf().add_subplot(121, projection='3d')
        ax.plot(loam[:,1], loam[:,2], loam[:,3], '.-m', markersize=0.5)
        jmp.decorate(ax, legend=['loam'])
        jmp.set_axes_equal(ax)

        ax = plt.gcf().add_subplot(122, projection='3d')
        ax.plot(gps[:,1], gps[:,2], gps[:,3], '.-m', markersize=0.5)
        ax.plot(l3[:,0], l3[:,1], l3[:,2], '.-g', markersize=0.5)
        jmp.decorate(ax, legend=['gps', 'loam_transformed'])
        jmp.set_axes_equal(ax)

        plt.show()
    return np.array(p)


def publish_transform(p):
    R = jma.rmat_of_euler(p[1:4])
    R2 = np.eye(4)
    R2[:3,:3] = R 
    #R1 = tf.transformations.euler_matrix(p[1], p[2], p[3], 'rxyz')
    #pdb.set_trace()
    q = tf.transformations.quaternion_from_matrix(R2)
    q_str = '{} {} {} {}'.format( *q)
    t_str = '{} {} {}'.format( *p[4:])
    cmd = 'rosrun tf static_transform_publisher {} {} map camera_init 100'.format(t_str, q_str)
    LOG.info( 'running {}'.format(cmd))
    subprocess.call(cmd, shell=True)

    
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=6, linewidth=600)
    p = find_transform(filename='gps_loam_2.npz', plot=False)
    publish_transform(p)
