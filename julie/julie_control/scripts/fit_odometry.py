#!/usr/bin/env python


import os, logging, math, numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tft
import pdb
import julie_misc.three_d_plot as jmp

def compute_enc_vel(enc_lw, enc_rw, enc_stamp):
    enc_vel_lw = enc_lw[1:] - enc_lw[:-1]
    enc_vel_rw = enc_rw[1:] - enc_rw[:-1]
    enc_dt = enc_stamp[1:] - enc_stamp[:-1]
    enc_vel_lw /= enc_dt
    enc_vel_rw /= enc_dt
    enc_vel_stamp = (enc_stamp[:-1] + enc_stamp[1:])/2
    return enc_vel_lw, enc_vel_rw, enc_vel_stamp, enc_dt


def compute_body_truth(truth_lvel, truth_rvel, truth_ori):
    n = len(truth_lvel)
    truth_lvel_b, truth_rvel_b = np.zeros((n, 3)), np.zeros((n, 3))
    for i in range(n):
        R_w2b = tft.quaternion_matrix(truth_ori[i])[:3,:3].T
        truth_lvel_b[i] = np.dot(R_w2b, truth_lvel[i])
        truth_rvel_b[i] = np.dot(R_w2b, truth_rvel[i])
    return truth_lvel_b, truth_rvel_b

def plot_body_truth(truth_stamp, truth_lvel_b, truth_rvel_b):
    plt.figure()
    ax=plt.subplot(3,2,1); plt.plot(truth_stamp, truth_lvel_b[:,0])
    jmp.decorate(ax, 'lvel body x')
    ax=plt.subplot(3,2,3); plt.plot(truth_stamp, truth_lvel_b[:,1])
    jmp.decorate(ax, 'lvel body y')
    ax=plt.subplot(3,2,5); plt.plot(truth_stamp, truth_lvel_b[:,2])
    jmp.decorate(ax, 'lvel body z')
    ax=plt.subplot(3,2,2); plt.plot(truth_stamp, truth_rvel_b[:,0])
    jmp.decorate(ax, 'rvel body x')
    ax=plt.subplot(3,2,4); plt.plot(truth_stamp, truth_rvel_b[:,1])
    jmp.decorate(ax, 'rvel body y')
    ax=plt.subplot(3,2,6); plt.plot(truth_stamp, truth_rvel_b[:,2])
    jmp.decorate(ax, 'rvel body z')

def interpolate(_d, _d_stamps, _stamps_1):
    _d_1 = np.zeros(len(_stamps_1))
    i = -1
    for i1, s1 in enumerate(_stamps_1):
        if i < len(_d_stamps)-1 and s1 >= _d_stamps[i+1]:
            i+=1
        if i==-1:
            _d_1[i1] = _d[0]
        elif i < len(_d_stamps)-1:
            _d_1[i1] = (s1-_d_stamps[i])/(_d_stamps[i+1]-_d_stamps[i])*(_d[i+1]-_d[i]) + _d[i]
        else:
            _d_1[i1] = _d[-1]
        #if i1 > 100: break
        #print s1, i, _d_stamps[i], _d_stamps[i+1], _d[i], _d[i+1], _d_1[i1]
    return _d_1


def fit(filename, plot=False):
    # read data file
    data =  np.load(filename)
    enc_lw, enc_rw = data['encoders_lw'], data['encoders_rw']
    enc_st, enc_stamp = data['encoders_st'], data['encoders_stamp']

    truth_pos, truth_ori = data['truth_pos'], data['truth_ori'] # xyzw
    truth_lvel, truth_rvel = data['truth_lvel'], data['truth_rvel']
    truth_stamp = data['truth_stamp']

    # compute truth in body frame 
    truth_lvel_b, truth_rvel_b = compute_body_truth(truth_lvel, truth_rvel, truth_ori)
    #plot_body_truth(truth_stamp, truth_lvel_b, truth_rvel_b)
    
    # differentiate rotary encoders
    enc_vel_lw, enc_vel_rw, enc_vel_stamp, enc_dt =  compute_enc_vel(enc_lw, enc_rw, enc_stamp)
    # interpolate
    truth_lvel_bx1 = interpolate(truth_lvel_b[:,0], truth_stamp, enc_vel_stamp)
    #plt.plot(truth_stamp, truth_lvel_b[:,0], '*')
    #plt.plot(enc_vel_stamp, truth_lvel_bx1, '.')
    
    # linear regression for linear vel
    enc_vel_avg = (enc_vel_lw + enc_vel_rw) / 2
    wheel_radius = np.mean(truth_lvel_bx1 / enc_vel_avg)
    #wheel_radius = 0.470/2
    odom_lvel = enc_vel_avg * wheel_radius

    wheel_base = 1.65
    enc_st_2 = (enc_st[1:] + enc_st[:-1])/2 # let's get the steering at the same time than rotary vel
    odom_rvel = odom_lvel*np.tan(enc_st_2)/ wheel_base
    
    # plot encoders position
    plt.figure()
    plt.plot(enc_stamp, enc_lw, '.')
    plt.plot(enc_stamp, enc_rw, '.')

    # plot encoders 
    plt.figure()
    ax = plt.subplot(2,1,1)
    plt.plot(truth_stamp, truth_lvel_b[:,0])
    plt.plot(enc_vel_stamp, odom_lvel)
    jmp.decorate(ax, 'linear velocity', ylab='m/s', legend=['thuth', 'odometry'])
    ax = plt.subplot(2,1,2)
    plt.plot(truth_stamp, truth_rvel_b[:,2])
    plt.plot(enc_vel_stamp, odom_rvel)
    jmp.decorate(ax, 'angular velocity', ylab='r/s', legend=['thuth', 'odometry'])
    
    plt.figure()
    ax = plt.subplot(2,1,1)
    plt.plot(enc_vel_stamp, enc_vel_lw, '.')
    plt.plot(enc_vel_stamp, enc_vel_rw, '.')
    plt.plot(enc_vel_stamp, enc_vel_avg, '.')
    jmp.decorate(ax, 'rotary encoders velocity', ylab='r', legend=['left rear wheel', 'right rear wheel', 'average rear wheels'])
    ax = plt.subplot(2,1,2)
    plt.plot(enc_stamp, enc_st, '.')
    jmp.decorate(ax, 'steering encoder', ylab='r')
                 
    plt.figure()
    #plt.plot(enc_vel_stamp, enc_dt)
    plt.hist(enc_dt, bins=100)
    jmp.decorate(plt.gca(), 'encoder delta timestamp')
    plt.show()
    
if __name__ == '__main__':
    np.set_printoptions(precision=6)
    fit('/tmp/foo.npz', plot=True)
