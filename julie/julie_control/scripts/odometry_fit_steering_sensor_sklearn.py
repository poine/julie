#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, logging, math, numpy as np, pickle
import matplotlib.pyplot as plt
import sklearn.neural_network

'''
  Fitting the steering sensor with a sklearn ANN
'''


import odometry_fit_steering_sensor as ofss

LOG = logging.getLogger('odometry_fit_steering_sensor_sklearn')

class AnnSteeringSensor:
    def __init__(self, epochs=100):
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
        self.ann = sklearn.neural_network.MLPRegressor(**params)

    def train(self, _inputs, _outputs):
        self.scaler = None
        self.ann.fit(_inputs , _outputs)
     
    def load(self, filename):
        LOG.info(' Loading ann from {}'.format(filename))
        with open(filename, "rb") as f:
            self.ann, self.scaler = pickle.load(f)

    def save(self, filename):
        LOG.info('  saving ann to {}'.format(filename))
        with open(filename, "wb") as f:
            pickle.dump([self.ann, self.scaler], f)

    def predict(self, _input):
        return self.ann.predict(_input)
            

def main(train, test, filename='/tmp/odometry_steering_sensor_sklearn.pkl', lim=math.pi/6, epochs=100):
    ss = AnnSteeringSensor(epochs=epochs)
    if train:
        nb_samples = int(1e5)
        steering_angles = np.random.uniform(low=-lim, high=lim, size=(nb_samples,1))
        steering_sensors =  ofss.BrokenDataSet.steering_sensor(steering_angles)
        ss.train(steering_sensors, steering_angles)
        ss.save(filename)
    else:
       ss.load(filename) 
    if test:
       true_angles = np.linspace(-lim, lim, 1000)
       steering_sensors = ofss.BrokenDataSet.steering_sensor(true_angles)
       predicted_angles = ss.predict(steering_sensors.reshape(-1, 1))
       ofss.plot_test_steering_sensor(steering_sensors, true_angles, predicted_angles, 'sklearn {}'.format(epochs))
       plt.show()
       
if __name__ == '__main__':
    np.set_printoptions(precision=6)
    main(train=False, test=True, epochs=8000)
