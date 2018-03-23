#!/usr/bin/env python


import sys, os, time, logging, math, numpy as np, pyproj
import PIL.Image, io
import goompy
import globalmaptiles
import pdb


tile_size = 256


def rad_of_deg(d): return d/180.*math.pi
#
# clicking in front of J building
# 43.563227, 1.481796
#
# https://developers.google.com/maps/documentation/static-maps/intro
# key='AIzaSyDwQSdWviORc05jmWP98iEU-jNpI5fYx9o'
#
# http://www.maptiler.org/google-maps-coordinates-tile-bounds-projection/
#

def rm_enu_to_ecef(lon, lat):
    '''lat: geodetic latitude'''
    clon, slon = math.cos(lon), math.sin(lon)
    clat, slat = math.cos(lat), math.sin(lat)
    return np.array([[-slon, -slat*clon, clat*clon ],
                     [ clon, -slat*slon, clat*slon ],
                     [ 0.,    clat,      slat      ]])

if sys.version_info[0] == 2:
    import urllib
    urlopen = urllib.urlopen
else:
    import urllib.request
    urlopen = urllib.request.urlopen
    
class LocalMapper:
    def __init__(self, cache_dir='/home/poine/google_map_cache'):
        self.cache_dir = cache_dir
        #self.lla0=[1.481796, 43.563227, 0.1951]
        #self.lla0, self.zoom, self.radius = [1.4835, 43.5639, 0.1951], 18, 180
        #self.lla0, self.zoom, self.radius = [1.4835, 43.5639, 0.1951], 19, 180
        #self.lla0, self.zoom, self.radius = [1.4829, 43.5645, 0.1951], 20, 140
        #self.lla0, self.zoom, self.radius, self.n_tiles = [1.4829, 43.5644, 0.1951], 21, None, 6
        self.lla0, self.zoom, self.radius, self.n_tiles = [1.4827, 43.5646, 0.1951], 22, None, 32
        self.lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
        self.ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
        self.ecef0 = np.array(pyproj.transform(self.lla, self.ecef, *self.lla0, radians=False))
        self.R_enu_to_ecef = rm_enu_to_ecef(rad_of_deg(self.lla0[0]), rad_of_deg(self.lla0[1]))

        #self.try_goompy()
        #self.try_myself()
        
    def make_map(self, latlon0, _z, _w, _h, _t='s'):
        tile_types={'roads only':'h',
                    'standard_roadmap':'m',
                    'terrain':'p',
                    'altered_roadmap':'r',
                    'satellite_only':'s',
                    'terrain_only':'t',
                    'hybrid':'y'}
        mercator = globalmaptiles.GlobalMercator()
        mx, my = mercator.LatLonToMeters( *latlon0 )
        print(mx, my)
        tx, ty = mercator.MetersToTile( mx, my, _z )
        gx, gy = mercator.GoogleTile(tx, ty, _z)
        print( "\tGoogle:", gx, gy)
        # python globalmaptiles.py 20 43.5646 1.4827
        #x, y, z, _t = 528606, 383039, 20, tile_types['satellite_only']
        # python globalmaptiles.py 21 43.5646 1.4827
        #x, y, z, _t = 1057213, 766078, 21
        # python globalmaptiles.py 22 43.5646 1.4827
        #x, y, z, _t = 2114426, 1532157, 22, tile_types['satellite_only']
        
        
        map_image = PIL.Image.new('RGB', (_w*tile_size, _h*tile_size))
        for i in range(_w):
            for j in range(_h):
                tile = self.fetch_tile_by_id(_t, gx+i, gy+j, _z)
                map_image.paste(tile, (i*tile_size, j*tile_size))
        map_image.save('map_{}_{}_{}_{}_{}_{}.png'.format(_t, _z, gx, gy, _w, _h))
            
    def fetch_tile_by_id(self, tile_type, x, y, z):
        filename = '{}/tile_{:s}_{:d}_{:d}_{:d}.png'.format(self.cache_dir, tile_type, z, x, y)
        if os.path.isfile(filename):
            tile = PIL.Image.open(filename)
        else:
            url = 'http://mt.google.com/vt/lyrs={:s}&x={:d}&y={:d}&z={:d}'.format(tile_type, x, y, z)
            print('fetching {}'.format(url))
            result = urlopen(url).read()
            tile = PIL.Image.open(io.BytesIO(result))
            if not os.path.exists(self.cache_dir):
                os.mkdir(self.cache_dir)
            tile.save(filename)
            time.sleep(0.25)
        return tile

        
    def try_goompy(self):
        WIDTH = 2560#1600#800 5120x1600
        HEIGHT = 1600#1000#500

        LATITUDE  = self.lla0[1] #37.7913838
        LONGITUDE = self.lla0[0] #-79.44398934
        ZOOM = self.zoom
        MAPTYPE = 'satellite'#roadmap'

        self.goompy = goompy.GooMPy(WIDTH, HEIGHT, LATITUDE, LONGITUDE, ZOOM, MAPTYPE, radius_meters=self.radius, default_ntiles=self.n_tiles, img_fmt='png')
        self.image = self.goompy.getImage()
        self.image.save('foo.png')

        self.goompy.bigimage.save('big_foo.png')
        print('{} {}'.format(self.goompy.northwest, self.goompy.southeast))
        #pdb.set_trace()


class Map:
    def __init__(self, filename):
        print('loading {}'.format(filename))
        tokens = os.path.splitext(filename)[0].split('_')
        self._type, self.zoom, self.gx, self.gy, self.wt, self.ht = tokens[1:]
        self.zoom, self.gx, self.gy = int(self.zoom), int(self.gx), int(self.gy)
        self.tx, self.ty = self.gx, (2**self.zoom - 1) - self.gy
        self.mercator = globalmaptiles.GlobalMercator()

    def latlon_to_pixels(self, lat, lon):
        mx, my = self.mercator.LatLonToMeters(lat, lon)
        print('meters ', mx, my)
        px, py = self.mercator.MetersToPixels(mx, my, self.zoom)
        print('pixels ', px, py)
        rx, ry = self.mercator.PixelsToRaster(px, py, self.zoom)
        print('raster ', rx, ry)
        mx, my = rx-self.tx*tile_size, ry-self.gy*tile_size 
        print('map ', mx, my)
        return mx, my

    def pixels_to_latlon(self, mx, my):
        rx, ry = mx + self.gx*tile_size, my+self.gy*tile_size 
        #print('raster ', rx, ry)
        px, py = self.mercator.RasterToPixel(rx, ry, self.zoom)
        #print('pixels ', px, py)
        mx, my = self.mercator.PixelsToMeters(px, py, self.zoom)
        #print('meters ', mx, my)
        lat, lon = self.mercator.MetersToLatLon(mx, my)
        return lat, lon
        
    def get_resolution(self):
        pass
    
def main(args):

    if 1:
        #latlon0 = 43.5646, 1.4827     # 528606_383039 
        #latlon0 = 43.563797, 1.481854 # 528604_383042
        #latlon0 = 43.564912, 1.480748 # 528600_383038
        latlon0 =  43.567259, 1.478873
        zoom, htiles, vtiles = 22, 22, 21
        LocalMapper().make_map(latlon0, zoom, htiles, vtiles)
    if 1:
        #m = Map('map_s_20_528606_383039_4_3.png')
        #print(m.latlon_to_pixels(43.564369, 1.483232)) # 571, 360
        #print(m.latlon_to_pixels(43.564000, 1.484387)) # 1433, 743

        m = Map('map_s_20_528604_383042_6_4.png')
        #print(m.latlon_to_pixels(43.563241, 1.481859)) # 56 754
        print(m.latlon_to_pixels(43.563811, 1.482785)) # 747, 166

        print(m.pixels_to_latlon(747, 166))
        
if __name__ == '__main__':
    main(sys.argv)
    
