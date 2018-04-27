#!/usr/bin/env python


import os, logging, math, numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tft
import pdb
import julie_misc.three_d_plot as jmp

def plot_velocities(ds, odom_lvel, odom_rvel):
    plt.figure()
    ax = plt.subplot(2,1,1)
    plt.plot(ds.truth_stamp, ds.truth_lvel_b[:,0])
    plt.plot(ds.enc_vel_stamp, odom_lvel)
    jmp.decorate(ax, 'linear velocity', ylab='m/s', legend=['thuth', 'odometry'])
    ax = plt.subplot(2,1,2)
    plt.plot(ds.truth_stamp, ds.truth_rvel_b[:,2])
    plt.plot(ds.enc_vel_stamp, odom_rvel)
    jmp.decorate(ax, 'angular velocity', ylab='r/s', legend=['thuth', 'odometry'])

def plot_positions(ds, odom_heading, odom_xy):
    plt.figure()
    ax=plt.subplot(2,2,1); plt.plot(ds.truth_stamp, ds.truth_heading_unwrapped)
    plt.plot(ds.enc_vel_stamp, odom_heading)
    jmp.decorate(ax, 'heading', legend=['truth', 'odom'])
    ax=plt.subplot(2,2,2); plt.plot(ds.truth_pos[:,0], ds.truth_pos[:,1])
    plt.plot(odom_xy[:,0], odom_xy[:,1])
    jmp.decorate(ax, '2D')

    heading_err =  np.rad2deg(odom_heading - ds.truth_heading_unwrapped1)
    ax=plt.subplot(4,1,3); plt.plot(ds.enc_vel_stamp, heading_err)
    jmp.decorate(ax, 'heading err', ylab='deg')

    pos_err = np.linalg.norm(odom_xy - ds.truth_xy1, axis=1)
    ax=plt.subplot(4,1,4); plt.plot(ds.enc_vel_stamp, pos_err)
    jmp.decorate(ax, 'err dist', ylab='m')
  
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


def plot_interpolation(stamp, d, stamp_1, d_1):
    plt.plot(stamp, d, '*')
    plt.plot(stamp_1, d_1, '.')
    
    
    
def interpolate(_d, _d_stamps, _stamps_1):
    # returns and interpolated version of _d at stamps  _stamps_1
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
    return _d_1


class DataSet:
    def __init__(self, filename):
        # read data file
        data =  np.load(filename)
        self.enc_lw, self.enc_rw = data['encoders_lw'], data['encoders_rw']
        self.enc_st, self.enc_stamp = data['encoders_st'], data['encoders_stamp']

        self.truth_pos, self.truth_ori = data['truth_pos'], data['truth_ori'] # xyzw
        self.truth_lvel, self.truth_rvel = data['truth_lvel'], data['truth_rvel']
        self.truth_stamp = data['truth_stamp']

        # compute truth in body frame 
        self.compute_body_truth()
    
        # differentiate rotary encoders
        self.compute_enc_vel()
        self.enc_st_2 = (self.enc_st[1:] + self.enc_st[:-1])/2 # let's get the steering at the same time than rotary vel

        # interpolate
        self.truth_lvel_bx1 = interpolate(self.truth_lvel_b[:,0], self.truth_stamp, self.enc_vel_stamp)
        self.truth_rvel_bz1 = interpolate(self.truth_rvel_b[:,2], self.truth_stamp, self.enc_vel_stamp)
        self.truth_heading_unwrapped1 = interpolate(self.truth_heading_unwrapped, self.truth_stamp, self.enc_vel_stamp)
        x1 = interpolate(self.truth_pos[:,0], self.truth_stamp, self.enc_vel_stamp)
        y1 = interpolate(self.truth_pos[:,1], self.truth_stamp, self.enc_vel_stamp)
        self.truth_xy1 = np.vstack((x1, y1)).T
        
    def compute_body_truth(self):
        n = len(self.truth_lvel)
        self.truth_lvel_b, self.truth_rvel_b = np.zeros((n, 3)), np.zeros((n, 3))
        self.truth_heading = np.zeros(n)
        for i in range(n):
            T_w2b = tft.quaternion_matrix(self.truth_ori[i])
            R_w2b = T_w2b[:3,:3].T
            self.truth_lvel_b[i] = np.dot(R_w2b, self.truth_lvel[i])
            self.truth_rvel_b[i] = np.dot(R_w2b, self.truth_rvel[i])
            self.truth_heading[i] = tft.euler_from_matrix(T_w2b, 'rzyx')[0]
        self.truth_heading_unwrapped = np.unwrap(self.truth_heading)

    def compute_enc_vel(self):
        self.enc_vel_lw = self.enc_lw[1:] - self.enc_lw[:-1]
        self.enc_vel_rw = self.enc_rw[1:] - self.enc_rw[:-1]
        self.enc_dt = self.enc_stamp[1:] - self.enc_stamp[:-1]
        self.enc_vel_lw /= self.enc_dt
        self.enc_vel_rw /= self.enc_dt
        self.enc_vel_stamp = (self.enc_stamp[:-1] + self.enc_stamp[1:])/2

        self.enc_vel_avg = (self.enc_vel_lw + self.enc_vel_rw) / 2
        self.enc_vel_dif =  self.enc_vel_lw - self.enc_vel_rw

    
        
            
def check_timestamps(enc_vel_stamp, enc_dt):
    plt.figure()
    #plt.plot(enc_vel_stamp, enc_dt)
    plt.hist(enc_dt, bins=100)
    jmp.decorate(plt.gca(), 'encoder delta timestamp')

def plot_encoders_vel(enc_vel_stamp, enc_vel_lw, enc_vel_rw, enc_vel_avg, enc_stamp, enc_st):
    # plot encoders velocities
    plt.figure()
    ax = plt.subplot(2,1,1)
    plt.plot(enc_vel_stamp, enc_vel_lw, '.')
    plt.plot(enc_vel_stamp, enc_vel_rw, '.')
    plt.plot(enc_vel_stamp, enc_vel_avg, '.')
    jmp.decorate(ax, 'rotary encoders velocity', ylab='r', legend=['left rear wheel', 'right rear wheel', 'average rear wheels'])
    ax = plt.subplot(2,1,2)
    plt.plot(enc_stamp, enc_st, '.')
    jmp.decorate(ax, 'steering encoder', ylab='r')

def plot_encoders(enc_stamp, enc_lw, enc_rw):
    # plot encoders position
    plt.figure()
    plt.plot(enc_stamp, enc_lw, '.')
    plt.plot(enc_stamp, enc_rw, '.')
 

def norm_angle(a):
    while a>math.pi: a -= 2*math.pi
    while a<-math.pi: a += 2*math.pi
    return a


class LinReg:

    def fit(self, ds, plot=False):

        #plot_body_truth(ds.truth_stamp, ds.truth_lvel_b, ds.truth_rvel_b)
        #plot_interpolation(ds.truth_stamp, ds.truth_lvel_b[:,0], ds.enc_vel_stamp, ds.truth_lvel_bx1)
        #plot_interpolation(ds.truth_stamp, ds.truth_rvel_b[:,2], ds.enc_vel_stamp, ds.truth_rvel_bz1)
        #plt.show()

        # linear regression for wheel_radius and wheel_base
        Y = np.concatenate([[_lvi, _rvi] for _lvi, _rvi in zip(ds.truth_lvel_bx1, ds.truth_rvel_bz1)])
        H = np.vstack([np.diag((reai, reai*math.tan(sti))) for reai, sti in zip(ds.enc_vel_avg, ds.enc_st_2)])
        w = np.dot(np.linalg.pinv(H), Y)
        self.wheel_radius = w[0]
        self.wheel_base = w[0]/w[1]
    
        wheel_radius_thruth, wheel_base_truth = 0.470/2, 1.65
        print('wr {:.4f} ({:.4f}) wb {:.4f} ({:.4f})'.format(self.wheel_radius, wheel_radius_thruth, self.wheel_base, wheel_base_truth))

    def compute(self, ds):
        odom_lvel = ds.enc_vel_avg * self.wheel_radius
        odom_rvel = odom_lvel*np.tan(ds.enc_st_2)/ self.wheel_base
        odom_heading, odom_xy = np.zeros(len(ds.enc_vel_stamp)), np.zeros((len(ds.enc_vel_stamp), 2))
        odom_heading[0], odom_xy[0] = ds.truth_heading[0], ds.truth_pos[0,:2]
        for i in range(1, len(ds.enc_vel_stamp)):
            odom_heading[i] = odom_heading[i-1] + odom_rvel[i-1]*ds.enc_dt[i-1]
            dxim1 = np.array([np.cos(odom_heading[i-1]), np.sin(odom_heading[i-1])])
            odom_xy[i] = odom_xy[i-1] + odom_lvel[i-1]*ds.enc_dt[i-1]*dxim1
        plot_velocities(ds, odom_lvel, odom_rvel)
        plot_positions(ds, odom_heading, odom_xy)
  
        #plot_encoders_vel(enc_vel_stamp, enc_vel_lw, enc_vel_rw, enc_vel_avg, enc_stamp, enc_st)
        #plot_encoders(enc_stamp, enc_lw, enc_rw)
        #check_timestamps(enc_vel_stamp, enc_dt)         
        plt.show()


class ANN:

    def fit(self, ds):
        pass

    
    
if __name__ == '__main__':
    np.set_printoptions(precision=6)
    ds = DataSet('/tmp/odom_data_1.npz')
    odom = LinReg()
    odom.fit(ds, plot=True)
    odom.compute(ds)
    
