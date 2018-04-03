import math, yaml


def rad_of_deg(d): return d/180.*math.pi
def deg_of_rad(r): return r/math.pi*180

class LTPFrame:
    '''
      Convert from ROS frame ( simple equirectangular projection ) to/from others
    '''

    def __init__(self, ref_filename):
        with open(ref_filename, 'r') as stream:
            ref_data = yaml.load(stream)
        print('ref_data {}'.format(ref_data))

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

    # checked with display_gps
    def ros_to_world(self, p_ros):
        lat = self.ref_lat + deg_of_rad(( self.crh*p_ros[0] + self.srh*p_ros[1])/self.radius_north)
        lon = self.ref_lon - deg_of_rad((-self.srh*p_ros[0] + self.crh*p_ros[1])/self.radius_east)
        alt = self.ref_alt + p_ros[2]
        return [lon, lat, alt]

    # checked with display_gps
    def world_to_ros(self, p_lla):
        lon, lat, alt = p_lla
        dlon_r, dlat_r = rad_of_deg(lon-self.ref_lon), rad_of_deg(lat-self.ref_lat)
        x_e, x_n = -dlon_r*self.radius_east, dlat_r*self.radius_north
        x, y = self.crh*x_n - self.srh*x_e, self.srh*x_n + self.crh*x_e
        z = alt-self.ref_alt
        return [x, y, z]
