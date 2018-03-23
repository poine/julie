#!/usr/bin/env python


import sys, os, time, logging, math, numpy as np
import pdb
# https://developers.google.com/kml/documentation/kml_tut

import test_01_make_google_map as gm

inkscape_str='''m 799.28572,2029.8571 -65,45.7143 -73.57143,53.5715 -65,47.8571 -18.57143,50.7143 22.14285,36.4286 42.14286,12.8571 45.71429,-14.2857 50.71428,-34.2857 219.28572,-159.2857 19.28571,-28.5715 -3.57143,-32.1428 -15,-27.8572 -28.57142,-22.1428 -33.57143,4.2857 -34.28572,22.1428 -57.14285,40.7143'''

if __name__ == '__main__':
    toks = inkscape_str.split(' ')
    vertices = []
    for t in toks:
        if len(t) == 1:
            print('command ',t)
        else:
            print('arg ', t)
            rel_pt = np.fromstring(t, sep=',')
            abs_pt = rel_pt if len(vertices)==0 else vertices[-1]+rel_pt
            vertices.append(abs_pt)
        
    print np.array(vertices)
    m = gm.Map('map_s_20_528595_383028_22_22.png')
    with open('foo.kml', 'w') as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write('<Document>\n')
        f.write('<name>Paths</name>\n')
        f.write('<Placemark>\n')
        f.write('<LineString>\n')
        f.write('<coordinates> ')
        for v in vertices:
            lat, lon = m.pixels_to_latlon(v[0], v[1])
            f.write('{},{},10\n'.format(lon, lat))
        f.write('</coordinates>\n')
        f.write('</LineString>\n')
        f.write('</Placemark>\n')
        f.write('</Document>\n')
        f.write('</kml>\n')
