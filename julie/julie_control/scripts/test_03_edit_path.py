#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import julie_control.two_d_guidance.path_editor as ped

def main(args):
    rospy.init_node('test_path_editor')
    _ped = ped.PathEditor(load_xml="/tmp/foo_path.xml", save="/tmp/foo2")
    _ped.save_path("/home/poine/work/julie/julie/julie_worlds/paths/enac_outdoor_south_east/path_J_1")
    #_ped = ped.PathEditor(load="/tmp/foo.npz")
    _ped.run()
    
if __name__ == '__main__':
    main(sys.argv)
