#!/usr/bin/env python


import sys, os, time, logging, math, numpy as np
import rospkg
import pdb

import julie_control.two_d_guidance as tdg

import two_d_guidance as tdg2

def make_oval(filename):
    c1, c2, r = np.array([17.5, -35.]), np.array([17.5, 13.]), 7.
    path = tdg.path_factory.make_oval_path(c1, c2, r)
    path.save(filename)

    
def main(args):
    jwd = rospkg.RosPack().get_path('julie_worlds')
    #make_oval(os.path.join(jwd, 'paths/test/oval_1.npz'))
    c1, c2, r = [-10, 0], [10, 0], 5
    if 1:
        path = tdg2.path_factory.make_oval_path(c1, c2, r)
        path.save(os.path.join(jwd, 'paths/test/oval_2.npz'))
    if 0:
        path = tdg2.make_fig_of_height_path2(10)
        path.save(os.path.join(jwd, 'paths/test/foe_1.npz'))
     
if __name__ == '__main__':
    main(sys.argv)
