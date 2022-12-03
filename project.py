#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm

"""
Function that calculates an elbow up Inverse Kinematic solution for the usv robot
"""
def ik_boat(y, z):
	# =================== Your code starts here ====================#
	l = 0.75 # for now, to be changed depending on length required 
	h = np.sqrt(y*y + z*z)
	
	ang0  = np.arcsin(z/h)
	ang1 = np.arccos(h/(2*l))

	theta0 = ang0 + ang1

	ang2 = np.pi - (2*ang1)

	theta1 = ang2 -np.pi/2
	
	thetas = [theta0, theta1] # angles are in radians
	return(thetas)
