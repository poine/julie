#!/usr/bin/env python


import os, logging, math, numpy as np
import matplotlib.pyplot as plt
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

def fit(filename, plot=False):
    data =  np.load(filename)
    enc_lw, enc_rw = data['encoders_lw'], data['encoders_rw']
    enc_st, enc_stamp = data['encoders_st'], data['encoders_stamp']

    truth_pos, truth_ori = data['truth_pos'], data['truth_ori']
    truth_lvel, truth_rvel = data['truth_lvel'], data['truth_rvel']
    truth_stamp = data['truth_stamp']
    
    enc_vel_lw, enc_vel_rw, enc_vel_stamp, enc_dt =  compute_enc_vel(enc_lw, enc_rw, enc_stamp)

    enc_vel_avg = (enc_vel_lw + enc_vel_rw) / 2

    thruth_lvel_norm =  np.linalg.norm(truth_lvel, axis=1)
    thruth_rvel_norm =  np.linalg.norm(truth_rvel, axis=1)
    #wheel_radius = thruth_lvel_norm / enc_vel_avg
    wheel_radius = 0.470/2
    odom_lvel = enc_vel_avg * wheel_radius
    wheel_base = 1.65
    enc_st_2 = (enc_st[1:] + enc_st[:-1])/2 # let's get the steering at the same time than rotary vel
    odom_rvel = np.abs(odom_lvel*np.tan(enc_st_2)/ wheel_base)
    
    #pdb.set_trace()
    
    plt.plot(enc_stamp, enc_lw, '.')
    plt.plot(enc_stamp, enc_rw, '.')

    plt.figure()
    ax = plt.subplot(2,1,1)
    plt.plot(truth_stamp, thruth_lvel_norm)
    plt.plot(enc_vel_stamp, odom_lvel)
    jmp.decorate(ax, 'linear velocity', ylab='m/s', legend=['thuth', 'odometry'])
    ax = plt.subplot(2,1,2)
    plt.plot(truth_stamp, thruth_rvel_norm)
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
