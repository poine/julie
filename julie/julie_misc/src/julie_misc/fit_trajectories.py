import numpy as np
import pdb

import julie_misc.algebra as jma

def interpolate(ti, t, p):
    if ti > t[-1]: return  p[-1] # do we want to extrapolate ?
    idx = np.argwhere(t>=ti)[0,0]
    #pdb.set_trace()
    return p[idx]


def synchronyze(traj_1, traj_2):
    t1, l1 = traj_1[:, 0], traj_1[:, 1:] 
    t2, l2 = traj_2[:, 0], traj_2[:, 1:] 

    l3 = np.zeros((len(t1),3))
    for i in range(len(t1)):
        l3[i] = interpolate(t1[i], t2, l2)
        
    return t1, l1, l3


def fit_horn(locs_rel, locs_abs):
  """Align two trajectories using Horn's method (closed-form)."""
  rel_mean, abs_mean =  locs_rel.mean(axis=0), locs_abs.mean(axis=0)
  rel_zero_centered, abs_zero_centered = locs_rel - rel_mean, locs_abs - abs_mean
  W = np.zeros((3, 3))
  for i in range(len(locs_rel)):
    W += np.outer(abs_zero_centered[i], rel_zero_centered[i])
  U,d,Vh = np.linalg.linalg.svd(W)
  S = np.matrix(np.identity( 3 ))
  if(np.linalg.det(U) * np.linalg.det(Vh)<0):
    S[2,2] = -1
  rot = U*S*Vh

  eulers = jma.euler_of_rmat(rot)
  #print "eulers: ", su.deg_of_rad(np.array(eulers))

  rel_rot =  (rot*np.array(rel_zero_centered).T).T
  dots, norms = 0., 0.
  for i in range(len(locs_rel)):
    dots += np.inner(abs_zero_centered[i], rel_rot[i])
    ni = np.linalg.norm(rel_zero_centered[i])
    norms += ni**2
  s = float(dots/norms)
  #print "scale: {:f} ".format(s)

  trans =  abs_mean.reshape((3,1)) - s*rot*rel_mean.reshape((3,1))
  #print "trans ", trans
  return [s, eulers[0], eulers[1], eulers[2], trans[0,0], trans[1,0], trans[2,0]]


def transform_mat_of_vect(v): return v[0]*jma.rmat_of_euler(v[1:4]), v[4:]

def transform(l, p):
    A, B = transform_mat_of_vect(p)    
    return np.array([np.dot(A, _l) + B for _l in l])


