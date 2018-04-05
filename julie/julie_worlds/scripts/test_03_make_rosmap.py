#!/usr/bin/env python


import sys, os, time, logging, math, numpy as np, yaml
import rospkg
import PIL.Image, PIL.ImageDraw

import pdb
import test_01_make_google_map as gm, julie_control.two_d_guidance as tdg, julie_worlds
'''

  How do I transform a google tile ( mercator ) to display in rviz ?

'''

def rad_of_deg(d): return d/180.*math.pi
def deg_of_rad(r): return r/math.pi*180


class RosMap():
    def __init__(self, size_px=(512, 256), resolution=0.05, origin=(0, 0, 0)):
        self.size_px = size_px
        self.resolution = resolution
        self.origin = np.array(origin)

    def world_to_pixel(self, p_w):
        p1 = (p_w[:2] - self.origin[:2])/self.resolution 
        px, py = int(np.round(p1[0])), int(np.round(self.size_px[1]-p1[1]-1))
        return px, py

    def pixel_to_world(self, p_px, alt=0.):
        p_wx, p_wy = (p_px[0]+0.5)*self.resolution, (self.size_px[1]-p_px[1]-1+0.5)*self.resolution
        return self.origin + [p_wx, p_wy, alt]

    def draw_test_map(self):
        self.map_image = PIL.Image.new('RGB', self.size_px)
        self.image_draw = PIL.ImageDraw.Draw(self.map_image)
        self.draw_line((0, 0), (5,0), (255, 255, 255), 2)
        self.draw_line((0, 0), (0,5), (128, 128, 128), 2)


    def draw_gm_tiles(self, ref_filename, gm_file):
        ltp = julie_worlds.LTPFrame(ref_filename)
        _gm = gm.Map(gm_file)
        gm_image = PIL.Image.open(gm_file)
        gm_pixels = gm_image.load()

        self.map_image = PIL.Image.new('RGB', self.size_px)
        self.image_draw = PIL.ImageDraw.Draw(self.map_image)
        map_pixels = self.map_image.load()
        for px in range(self.size_px[0]):
            for py in range(self.size_px[1]):
                p_ros = rm.pixel_to_world([px, py], 0)
                p_lla = ltp.ros_to_world(p_ros)  # position in world frame
                px_gm = np.round(_gm.latlon_to_pixels(p_lla[1], p_lla[0])).astype(int)
                map_pixels[px, py] = gm_pixels[px_gm[0], px_gm[1]]

    def draw_line(self, p1, p2, color, width):
        def as3d(_p): return [_p[0], _p[1], 0]
        p1x, p1y = self.world_to_pixel(as3d(p1))
        p2x, p2y = self.world_to_pixel(as3d(p2))
        self.image_draw.line([(p1x, p1y), (p2x, p2y)], fill=color, width=width)
        
    def draw_path(self, path_filename):
        _path = tdg.Path(load=path_filename)
        for i in range(len(_path.points)-1):
            self.draw_line(_path.points[i], _path.points[i+1], (255, 255, 255), 2)
                
    def save(self, map_dir, map_name):
        img_filename = os.path.join(map_dir, '{}.png'.format(map_name))
        self.map_image.save(img_filename)
        self.write_yaml(map_dir, map_name)

    def write_yaml(self, map_dir, map_name):
        yaml_output_file = os.path.join(map_dir, '{}.yaml'.format(map_name))
        with open(yaml_output_file, 'w') as stream:
            stream.write('image: {}.png\n'.format(ros_ref_name))
            stream.write('resolution: {}\n'.format(res))
            stream.write('origin: {}\n'.format(origin))
            stream.write('negate: 0\n')
            stream.write('occupied_thresh: 0.65\n')
            stream.write('free_thresh: 0.196\n')
  
if __name__ == '__main__':
    ros_ref_name = 'enac_outdoor_south_east'
    jw_dir = rospkg.RosPack().get_path('julie_worlds')
    ref_filename = os.path.join(jw_dir, 'config/ref_{}.yaml'.format(ros_ref_name))
    gm_filename = os.path.join(jw_dir, 'gmaps/map_s_20_528595_383028_22_22.png')
    
    # From map server documentation:
    #   origin : The 2-D pose of the lower-left pixel in the map, as (x, y, yaw),
    #            with yaw as counterclockwise rotation (yaw=0 means no rotation).
    #            Many parts of the system currently ignore yaw.

    res, origin = 0.05, [-75., -75., 0.]
    rl_size = np.array([150., 150.])
    px_size = (rl_size/res).astype(int)
    rm = RosMap(px_size, res, origin)
    #rm.draw_test_map()
    rm.draw_gm_tiles(ref_filename, gm_filename)
    path_filename = os.path.join(jw_dir, 'paths/enac_outdoor_south_east/path_J_1.npz')
    rm.draw_path(path_filename)
    
    map_dir, map_name = os.path.join(jw_dir, 'maps/'), ros_ref_name
    rm.save(map_dir, map_name)
    
 
