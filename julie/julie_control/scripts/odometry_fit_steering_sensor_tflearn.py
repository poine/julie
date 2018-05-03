#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, logging, math, numpy as np, pickle
import matplotlib.pyplot as plt

import tensorflow as tf, tflearn
import pdb

'''
  Fitting the steering sensor with a tflearn ANN
'''


import odometry_fit_steering_sensor as ofss

LOG = logging.getLogger('odometry_fit_steering_sensor_tflearn')

class AnnSteeringSensor:
    def __init__(self):
        inputs = tflearn.input_data(shape=[None, 1], dtype=tf.float32)
        self.layer1 = tflearn.fully_connected(inputs, 29, activation='relu', regularizer='L2', weight_decay=0.001)
        out = tflearn.fully_connected(self.layer1, 1, activation='linear', regularizer='L2', weight_decay=0.001)
        reg = tflearn.regression(out, optimizer='sgd', loss='mean_square', learning_rate=0.001, metric=None)
        self.model = tflearn.DNN(reg,
                                 tensorboard_dir='/tmp/odom_tflearn_logs/',
                                 best_checkpoint_path='/tmp/odom_best',
                                 checkpoint_path='/tmp/odom_current')

    def train(self, _inputs, _outputs, epochs):
        self.model.fit(_inputs, _outputs, n_epoch=epochs,
                       batch_size=64, show_metric=True, validation_set=0.1)

    def save(self, filename):
        self.model.save(filename)

    def load(self, filename):
        self.model.load(filename)

    def predict(self, _inputs):
        return self.model.predict(_inputs)
        
def main(train, test, lim=math.pi/6, epochs=100):
    filename='/tmp/odometry_steering_sensor_tflearn.pkl'
    ss = AnnSteeringSensor()
    if train:
        nb_samples = int(1e5)
        steering_angles = np.random.uniform(low=-lim, high=lim, size=(nb_samples,1))
        steering_sensors =  ofss.BrokenDataSet.steering_sensor(steering_angles)
        ss.train(steering_sensors, steering_angles, epochs)
        pdb.set_trace()
        ss.save(filename)
    else:
        ss.load(filename)
    if test:
        true_angles = np.linspace(-lim, lim, 1000).reshape(-1, 1)
        steering_sensors = ofss.BrokenDataSet.steering_sensor(true_angles)
        predicted_angles = ss.predict(steering_sensors.reshape(-1, 1))
        ofss.plot_test_steering_sensor(steering_sensors, true_angles, predicted_angles, 'sklearn {}'.format(epochs))
        plt.show()
       
if __name__ == '__main__':
    np.set_printoptions(precision=6)
    main(train=True, test=True, epochs=10)
