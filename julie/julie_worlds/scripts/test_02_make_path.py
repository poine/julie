#!/usr/bin/env python


import sys, os, time, logging, math, numpy as np
import rospkg
import pdb
# https://developers.google.com/kml/documentation/kml_tut

import test_01_make_google_map as gm
import test_03_make_rosmap as rm
import two_d_guidance

def inkscape_path_to_lla(path_str, gm_map_path):
    ''' convert inkscape svg path to latlon '''
    toks = path_str.split(' ')
    vertices_pixel = []
    for t in toks:
        if len(t) == 1:
            print('command ',t)
        else:
            print('arg ', t)
            rel_pt = np.fromstring(t, sep=',')
            abs_pt = rel_pt if len(vertices_pixel)==0 else vertices_pixel[-1]+rel_pt
            vertices_pixel.append(abs_pt)
    print np.array(vertices_pixel)
    m = gm.Map(gm_map_path)

    vertices_lla = []
    for v in vertices_pixel:
        v_lat, v_lon = m.pixels_to_latlon(v[0], v[1])
        vertices_lla.append([v_lon, v_lat, 0])
    return vertices_lla
    
def write_kml(vertices_lla, kml_path):
    with open(kml_path, 'w') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('<Document>\n')
        f.write('<name>Paths</name>\n')
        f.write('<Placemark>\n')
        f.write('<LineString>\n')
        f.write('<coordinates> ')
        for lla in vertices_lla:
            f.write('{},{},10\n'.format(lla[0], lla[1]))
        f.write('</coordinates>\n')
        f.write('</LineString>\n')
        f.write('</Placemark>\n')
        f.write('</Document>\n')
        f.write('</kml>\n')

inkscape_str_path_A='''m 799.28572,2029.8571 -65,45.7143 -73.57143,53.5715 -65,47.8571 -18.57143,50.7143 22.14285,36.4286 42.14286,12.8571 45.71429,-14.2857 50.71428,-34.2857 219.28572,-159.2857 19.28571,-28.5715 -3.57143,-32.1428 -15,-27.8572 -28.57142,-22.1428 -33.57143,4.2857 -34.28572,22.1428 -57.14285,40.7143'''

inkscape_str_path_J='''m 2313.2493,4369.3093 179.8072,256.5788 36.3655,16.1624 36.3655,8.0812 511.1372,-367.6955 22.2233,-42.4264 -16.1624,-52.5279 -268.7006,-365.6753 -62.6295,-6.0609 -143.4416,94.9544 -6.0609,42.4264 292.9442,389.9189 0,36.3654 -323.2488,234.3554 -30.3046,-2.0203 -38.3858,-14.1421 -385.8783,-531.3402 -24.2436,0 -171.726,119.198 -10.1015,28.2842 14.1422,24.2437 20.203,6.0609 155.5635,-109.0965 30.3046,2.0203 169.7056,220.2133'''


if __name__ == '__main__':
    gm_map_path = 'map_s_20_528595_383028_22_22.png'
    vertices_lla = inkscape_path_to_lla(inkscape_str_path_J, gm_map_path)
    if 0:
        write_kml(vertices_lla, 'path_J.kml')
    if 1:
        print vertices_lla
        ros_ref_name = 'enac_outdoor_south_east'
        rospack = rospkg.RosPack()
        jwd = rospack.get_path('julie_worlds')
        ref_filename = os.path.join(jwd, 'config/ref_{}.yaml'.format(ros_ref_name))
        rf = rm.RosFrame(ref_filename)
        p_ros = np.array([rf.world_to_ros(_lla) for _lla in vertices_lla])
        #db.set_trace()
        # that sucks...
        
        _p = two_d_guidance.Path(points=p_ros[:,:2], headings=np.zeros(len(p_ros)))
        _p.save('/tmp/foo')
        #print  p_ros
        
