import math, numpy as np

def rad_of_deg(d): return d/180.*math.pi

def deg_of_rad(r): return r*180./math.pi

def list_of_xyz(p): return [p.x, p.y, p.z]

def list_of_xyzw(q): return [q.x, q.y, q.z, q.w]
