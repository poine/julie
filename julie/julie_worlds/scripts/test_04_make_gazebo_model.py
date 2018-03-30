#!/usr/bin/env python


import sys, os, time, logging, math, numpy as np, yaml
import rospkg
import PIL.Image, PIL.ImageDraw

import pdb
import test_01_make_google_map as gm, test_03_make_rosmap as rm, julie_control.two_d_guidance as tdg
'''

  How do I transform a google tile ( mercator ) to display in gazebo ?

'''


def write_config(name, outfile):
    with open(outfile, 'w') as f:
      f.write('''<?xml version="1.0"?>
<model>
  <name>{}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>Antoine Drouin</name>
    <email>poinix@gmail.com</email>
  </author>

  <description>
    A textured ground plane.
  </description>
</model>
'''.format(name))


def write_sdf(model_name, outfile, size):
    with open(outfile, 'w') as f:
        size_str = '{} {}'.format(size[0], size[1])
        f.write('''<?xml version="1.0"?>
<sdf version="1.6">
<model name="{}">
  <pose>0 0 0.5 0 0 0</pose>
  <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{}</size>
          </plane>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>100</mu>
              <mu2>50</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{}</size>
          </plane>
        </geometry>
        <material>
          <script>
            <uri>model://{}/materials/scripts</uri>
            <uri>model://{}/materials/textures</uri>
	    <name>EnacFull/Image</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''.format(model_name, size_str, size_str, model_name, model_name))


def make_texture(model_name, model_dir, texture_size, texture_resolution, gm_file, ref_filename):
    texture_name = 'texture_{}.png'.format(model_name)
    script_dir = os.path.join(model_dir, 'materials/scripts/')
    if not os.path.exists(script_dir):
         os.makedirs(script_dir)
    outfile =  os.path.join(script_dir, '{}.material'.format(model_name))
    with open(outfile, 'w') as f:
        f.write('''material EnacFull/Image
{{
  technique
  {{
    pass
    {{
      ambient 0.5 0.5 0.5 1.0
      diffuse 1.0 1.0 1.0 1.0
      specular 0.0 0.0 0.0 1.0 0.5

      texture_unit
      {{
        texture {}
        filtering trilinear
        }}
      }}
   }}
 }}
        '''.format(texture_name))
        
    texture_dir = os.path.join(model_dir, 'materials/textures/')
    if not os.path.exists(texture_dir):
        os.makedirs(texture_dir)
    outfile =  os.path.join(texture_dir, texture_name)
    texture_image = PIL.Image.new('RGB', texture_size)
    texture_pixels = texture_image.load()

    _gm = gm.Map(gm_file)
    gm_image = PIL.Image.open(gm_file)
    gm_pixels = gm_image.load()
    _rm = rm.RosFrame(ref_filename)
    def pixel_to_world(px, py): return  [px/texture_resolution, py/texture_resolution, 0]

    for px in range(texture_size[0]):
        for py in range(texture_size[1]):
            p_ros = pixel_to_world(px, py)   # position in local world
            p_lla = _rm.ros_to_world(p_ros)  # position in gnss frame
            p_gm = np.round(_gm.latlon_to_pixels(p_lla[1], p_lla[0])).astype(int) # pixel in map tile
            print p_ros, p_lla, p_gm
            texture_pixels[px, py] = gm_pixels[p_gm[0], p_gm[1]]
    
    texture_image.save(outfile) 
    
   
def create_model(model_name, ref_name, model_size):
    jwd = rospkg.RosPack().get_path('julie_worlds')
    gm_file = os.path.join(jwd, 'gmaps/map_s_20_528595_383028_22_22.png')
    ref_filename = os.path.join(jwd, 'config/ref_{}.yaml'.format(ref_name))
    
    jgd = rospkg.RosPack().get_path('julie_gazebo')
    model_dir = os.path.join(jgd, 'models/', model_name)
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)
    
    write_config('enac ground plane', os.path.join(model_dir, 'model.config'))
    write_sdf(model_name, os.path.join(model_dir, 'model.sdf'), model_size)
    texture_resolution = 10. # pixel per meter
    texture_size = np.round(model_size*texture_resolution).astype(int)
    make_texture(model_name, model_dir, texture_size, texture_resolution, gm_file, ref_filename)
    
if __name__ == '__main__':
    ref_name = 'enac_outdoor_south_east'
    model_name = '{}_ground_plane'.format(ref_name)
    model_size = np.array([150., 150.])
    create_model(model_name, ref_name, model_size)
