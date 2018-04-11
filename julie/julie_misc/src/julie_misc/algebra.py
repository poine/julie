

import math, numpy as np

# remove this shit and use tranformations lib instead

e_phi   = 0
e_theta = 1
e_psi   = 2
e_size  = 3

def euler_of_rmat(r):
    phi   = math.atan2( r[1,2], r[2,2])
    theta = -math.asin( r[0,2])
    psi   = math.atan2( r[0,1], r[0,0])
    return phi, theta, psi

def rmat_of_euler(e):
    cphi = math.cos(e[e_phi]);     sphi = math.sin(e[e_phi])
    ctheta = math.cos(e[e_theta]); stheta = math.sin(e[e_theta])
    cpsi = math.cos(e[e_psi]);     spsi = math.sin(e[e_psi])
    return np.array([[ctheta*cpsi                 , ctheta*spsi                 , -stheta],
                     [sphi*stheta*cpsi - cphi*spsi, sphi*stheta*spsi + cphi*cpsi, sphi*ctheta],
                     [cphi*stheta*cpsi + sphi*spsi, cphi*stheta*spsi - sphi*cpsi, cphi*ctheta]])
