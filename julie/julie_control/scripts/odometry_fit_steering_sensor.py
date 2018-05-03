#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
let's see what having an incorrect wheel angle brings...
'''


import os, logging, math, numpy as np
import matplotlib.pyplot as plt
import sklearn.neural_network
import tensorflow as tf, tflearn

import pdb
import julie_misc.three_d_plot as jmp
import fit_odometry as fod

''' DataSet with a measurement error on the wheel angle '''
class BrokenDataSet(fod.DataSet):

    @staticmethod
    def steering_sensor(steering_angle):
        return np.sin(2.5*steering_angle)+0.1

    def __init__(self, filename):
        fod.DataSet.__init__(self, filename)
        self.enc_st_m = BrokenDataSet.steering_sensor(self.enc_st)
        # get the steering at the same time than rotary vel
        self.enc_st_m2 = (self.enc_st_m[1:] + self.enc_st_m[:-1])/2 

def plot_steering_sensor(lim=math.pi/6):
    steering_angle = np.linspace(-lim, lim, 100)
    steering_sensor = BrokenDataSet.steering_sensor(steering_angle)
    plt.plot(np.rad2deg(steering_angle), steering_sensor)
    jmp.decorate(plt.gca(), xlab='angle (deg)', ylab='sensor')
    


def sklearn_find_steering_sensor(lim=math.pi/6, nb_samples=int(1e5), epochs=100):
    steering_angles = np.random.uniform(low=-lim, high=lim, size=(nb_samples,1))
    steering_sensors = BrokenDataSet.steering_sensor(steering_angles)

    params = {
        'hidden_layer_sizes':(29),     # 
        'activation':'relu',           # ‘identity’, ‘logistic’, ‘tanh’, ‘relu’
        'solver': 'sgd',               # ‘lbfgs’, ‘sgd’, ‘adam’
        'verbose': True, 
        'random_state':1, 
        'max_iter':epochs, 
        'tol':1e-16,
        'warm_start': True
    }

    ann = sklearn.neural_network.MLPRegressor(**params)
    tr_input , tr_output = steering_sensors, steering_angles
    ann.fit(tr_input , tr_output)

    pdb.set_trace()
    
    steering_angles = np.linspace(-lim, lim, 1000)
    steering_sensors = BrokenDataSet.steering_sensor(steering_angles)
    predicted_angles = ann.predict(steering_sensors.reshape(-1, 1))
    plot_test_steering_sensor(steering_sensors, steering_angles, predicted_angles, 'tflearn {}'.format(epochs))



def tflearn__find_steering_sensor(lim=math.pi/6, nb_samples=int(1e5), epochs=100):
    inputs = tflearn.input_data(shape=[None, 1], dtype=tf.float32)

    net = tflearn.fully_connected(inputs, 29, activation='relu', regularizer='L2')#, weights_init=_w_init, bias_init=_b_init)
    #net = tflearn.layers.normalization.batch_normalization(net)
    #net = tflearn.activations.relu(net)
    out = tflearn.fully_connected(net, 1, activation='linear')

    net = tflearn.regression(out, optimizer='sgd', loss='mean_square', learning_rate=0.001, metric=None)
    model = tflearn.DNN(net,
                        tensorboard_dir='/tmp/odom_tflearn_logs/',
                        best_checkpoint_path='/tmp/odom_best',
                        checkpoint_path='/tmp/odom_current')

    steering_angles = np.random.uniform(low=-lim, high=lim, size=(nb_samples,1))
    steering_sensors = BrokenDataSet.steering_sensor(steering_angles)
    training_input , training_output = steering_sensors, steering_angles
    model.fit(training_input, training_output, n_epoch=epochs, batch_size=64, show_metric=True, validation_set=0.1)

    steering_angles = np.linspace(-lim, lim, 1000).reshape(-1, 1)
    steering_sensors = BrokenDataSet.steering_sensor(steering_angles)
    predicted_angles = model.predict(steering_sensors.reshape(-1, 1))
    plot_test_steering_sensor(steering_sensors, steering_angles, predicted_angles, 'sklearn {}'.format(epochs))

# float64 problem, but that does not seem to be the issue
# had fixed it by hacking tflear/layers/core.py and forcing float64... wtf!!!
def tflearn__find_steering_sensor2(lim=math.pi/6, nb_samples=int(1e5), epochs=100):
    inputs = tflearn.input_data(shape=[None, 1], dtype=tf.float64)
    true_output = tf.placeholder(shape=(None, 1), dtype=tf.float64)
    
    _b_init = tflearn.initializations.uniform(dtype=tf.float64)
    _w_init = tflearn.initializations.uniform(dtype=tf.float64)
    net = tflearn.fully_connected(inputs, 29)#, bias_init=_b_init, weights_init=_w_init)
    out = tflearn.fully_connected(net, 1, activation='linear')
    net = tflearn.regression(out, placeholder=true_output, optimizer='sgd', loss='mean_square', learning_rate=0.001, metric=None)
    model = tflearn.DNN(net,
                        tensorboard_dir='/tmp/odom_tflearn_logs/',
                        best_checkpoint_path='/tmp/odom_best',
                        checkpoint_path='/tmp/odom_current')

    steering_angles = np.random.uniform(low=-lim, high=lim, size=(nb_samples,1))
    steering_sensors = BrokenDataSet.steering_sensor(steering_angles)
    training_input , training_output = steering_sensors, steering_angles
    model.fit(training_input, training_output, n_epoch=epochs, batch_size=64, show_metric=True, validation_set=0.1)


def tf__find_steering_sensor():
    pass

    

    
def plot_test_steering_sensor(sensors, angles, predicted_angles, txt):
    plt.suptitle(txt)
    ax = plt.subplot(2, 1, 1)
    plt.plot(np.rad2deg(angles), sensors)
    plt.plot(np.rad2deg(predicted_angles), sensors)
    jmp.decorate(ax, xlab='angle (deg)', ylab='sensor', title='sensor', legend=['truth', 'ann'])
    ax = plt.subplot(2, 2, 3)
    residuals_deg = np.rad2deg(predicted_angles - angles)
    plt.plot(np.rad2deg(angles), residuals_deg)
    jmp.decorate(ax, xlab='angle (deg)', ylab='residuals (deg)', title='residual')
    ax = plt.subplot(2, 2, 4)
    plt.hist(residuals_deg, bins=100)
    mu, sigma = np.mean(residuals_deg), np.std(residuals_deg)
    plt.legend(['$\mu$ {:.2e}\n$\sigma$ {:.2e}'.format(mu, sigma)])


    
    
if __name__ == '__main__':
    np.set_printoptions(precision=6)
    #plot_steering_sensor()
    sklearn_find_steering_sensor(epochs=200)
    #tflearn__find_steering_sensor(epochs=100)
    #tflearn__find_steering_sensor2(epochs=100)
    #ds = BrokenDataSet('/tmp/odom_data_1.npz')
    plt.show()
