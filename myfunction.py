# -*- coding: utf-8 -*-
"""
Created on Tue Nov 01 13:54:05 2016

@author: liuqi
"""

import numpy as np

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    # assert v2_u is always (x,0)
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    t_sign = 1.0
    if(v1_u[1]>0):
        t_sign = -1.0
    return t_sign*np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))