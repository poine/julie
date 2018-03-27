#!/usr/bin/env python


import sys, os, time, logging, math, numpy as np, yaml
import rospkg
import PIL.Image

import pdb
import test_01_make_google_map as gm
'''

  How do I transform a google tile ( mercator ) to display in gazebo/rviz ?

'''

def rad_of_deg(d): return d/180.*math.pi
def deg_of_rad(r): return r/math.pi*180


class RosMap():
    def __init__(self, size_px=(512, 256), resolution=0.05, origin=(0, 0, 0)):
        self.size_px = size_px
        self.resolution = resolution
        self.origin = np.array(origin)

    def world_to_pixel(self, p_w):
        p1 = (p_w - self.origin)/self.resolution
        print p_w, '->', p1
        return p1[0], self.size_px[1]-p1[1]-1

    def pixel_to_world(self, p_px, alt=0.):
        p1 = np.array(p_px)*self.resolution
        return self.origin + [p1[0], p1[1], alt]

    def save(self, name):
        pass

    
class RosFrame():
    '''
     Convert from ROS frame ( simple equirectangular projection ) to/from others
    '''

    def __init__(self, ref_filename):
        with open(ref_filename, 'r') as stream:
            ref_data = yaml.load(stream)
        print ref_data
        
        self.ref_lat, self.ref_lon, self.ref_alt, self.ref_heading = [ref_data[s] for s in ['lat', 'lon', 'alt', 'heading']]
        rhr = rad_of_deg(self.ref_heading)
        self.crh, self.srh = math.cos(rhr), math.sin(rhr)
        equatorial_radius = 6378137.0
        flattening = 1.0/298.257223563
        excentrity2 = 2*flattening - flattening*flattening
        temp = 1.0 / (1.0 - excentrity2 * math.sin(rad_of_deg(self.ref_lat)) * math.sin(rad_of_deg(self.ref_lat)))
        prime_vertical_radius = equatorial_radius * math.sqrt(temp)
        self.radius_north = prime_vertical_radius * (1 - excentrity2) * temp
        self.radius_east  = prime_vertical_radius * math.cos(rad_of_deg(self.ref_lat))

    def ros_to_world(self, p_ros):
        lat = self.ref_lat + deg_of_rad(( self.crh*p_ros[0] + self.srh*p_ros[1])/self.radius_north)
        lon = self.ref_lon - deg_of_rad((-self.srh*p_ros[0] + self.crh*p_ros[1])/self.radius_east)
        alt = self.ref_alt + p_ros[2]
        return [lon, lat, alt]

    def world_to_ros(self, p_lla):
        lon, lat, alt = p_lla
        dlon_r, dlat_r = rad_of_deg(lon-self.ref_lon), rad_of_deg(lat-self.ref_lat)
        x_e, x_n = -dlon_r*self.radius_east, dlat_r*self.radius_north
        x, y = self.crh*x_n - self.srh*x_e, self.srh*x_n + self.crh*x_e
        #x = rad_of_deg(lat-self.ref_lat)*self.radius_north
        #y = -rad_of_deg(lon-self.ref_lon)*self.radius_east
        z = alt-self.ref_alt
        return [x, y, z]

    def make_ros_tile(self, px_size, res, origin, outfile, gm_file):
        _gm = gm.Map(gm_file)
        gm_image = PIL.Image.open(gm_file)
        gm_pixels = gm_image.load()

        print('ros map pixel size: {}'.format(px_size))
        rm = RosMap(px_size, res, origin)
        ros_map_image = PIL.Image.new('RGB', px_size)
        ros_map_pixels = ros_map_image.load()
        for px in range(px_size[0]):
            for py in range(px_size[1]):
                p_ros = rm.pixel_to_world([px, py], 0)
                p_lla = self.ros_to_world(p_ros)  # position in world frame
                # FIXME - not same pixels between google and ros....
                #print('ros px: {},{}'.format(px, py))
                px_gm = np.round(_gm.latlon_to_pixels(p_lla[1], p_lla[0])).astype(int)
                ros_map_pixels[px_size[1]-1-py, px_size[0]-1-px] = gm_pixels[px_gm[0], px_gm[1]]
        ros_map_image.save(outfile)
    
    def make_ros_testmap(self, size_px, res, origin, outfile, gm_file):
        rm = RosMap(size_px, res, origin)
        ros_map_image = PIL.Image.new('RGB', px_size)
        ros_map_pixels = ros_map_image.load()
        for i in range(0, 6):
            px, py = rm.world_to_pixel([i, 0, 0])
            ros_map_pixels[px, py] = (255,255,255)
            px, py = rm.world_to_pixel([0, i, 0])
            ros_map_pixels[px, py] = (128,128,128)
        ros_map_image.save(outfile) 

if __name__ == '__main__':
    ros_ref_name = 'enac_outdoor_south_east'

    rospack = rospkg.RosPack()
    jwd = rospack.get_path('julie_worlds')
    ref_filename = os.path.join(jwd, 'config/ref_{}.yaml'.format(ros_ref_name))

    gm_file = os.path.join(jwd, 'gmaps/map_s_20_528595_383028_22_22.png')
    
    # From map server documentation:
    #   origin : The 2-D pose of the lower-left pixel in the map, as (x, y, yaw),
    #            with yaw as counterclockwise rotation (yaw=0 means no rotation).
    #            Many parts of the system currently ignore yaw.

    res, origin = 0.05, [-75., -75., 0.]
    rl_size = np.array([150., 150.])
    px_size = (rl_size/res).astype(int)
    rf = RosFrame(ref_filename)
    ros_map_img_filename = os.path.join(jwd, 'maps/{}.png'.format(ros_ref_name))
    rf.make_ros_tile(px_size, res, origin, outfile=ros_map_img_filename, gm_file=gm_file)
    
    yaml_output_file = os.path.join(jwd, 'maps/{}.yaml'.format(ros_ref_name))
    with open(yaml_output_file, 'w') as stream:
        stream.write('image: {}.png\n'.format(ros_ref_name))
        stream.write('resolution: {}\n'.format(res))
        stream.write('origin: {}\n'.format(origin))
        stream.write('negate: 0\n')
        stream.write('occupied_thresh: 0.65\n')
        stream.write('free_thresh: 0.196\n')
