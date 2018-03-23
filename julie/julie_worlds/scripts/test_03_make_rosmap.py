#!/usr/bin/env python


import sys, os, time, logging, math, numpy as np, yaml
import rospkg

import pdb
import test_01_make_google_map as gm
'''

make local map

The conversion between gazebo coordinates and WGS84 is done using a simple equirectangular projection

https://en.wikipedia.org/wiki/Equirectangular_projection


'''


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    ref_filename = os.path.join(rospack.get_path('julie_worlds'), 'config/ref_enac_outdoor_1.yaml')
    with open(ref_filename, 'r') as stream:
        ref_data = yaml.load(stream)

    print ref_data
    m = gm.Map(os.path.join(rospack.get_path('julie_worlds'),'gmaps/map_s_20_528595_383028_22_22.png'))
    
    ref_pixels = m.latlon_to_pixels(ref_data['lat'], ref_data['lon'])
    print ref_pixels

    # origin : The 2-D pose of the lower-left pixel in the map, as (x, y, yaw),
    # with yaw as counterclockwise rotation (yaw=0 means no rotation). Many parts of the system currently ignore yaw.
    
    yaml_output_file = os.path.join(rospack.get_path('julie_worlds'), 'maps/enac_outdoor_1.yaml')
    with open(yaml_output_file, 'w') as stream:
        stream.write('image: enac_outdoor_1.png\n')
        stream.write('resolution: 0.005\n')
        stream.write('origin: [0., 0., 0.]\n')
        stream.write('negate: 0\n')
        stream.write('occupied_thresh: 0.65\n')
        stream.write('free_thresh: 0.196\n')
