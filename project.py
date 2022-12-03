
#!/usr/bin/env python3
import numpy as np
from scipy.linalg import expm

"""
Function that calculates an elbow up Inverse Kinematic solution for the usv robot
"""

def ik_boat(y, z):	

l = 0.75  # for now, to be changed depending on length required 

suct_gripper = 0.06
h = np.sqrt(y*y + z*z)
ang0  = np.arcsin(z/h)
ang1 = np.arccos(((h*h)-(suct_gripper*suct_gripper)-(2*suct_gripper*l))/(2*l*h))

theta0 = ang0 + ang1

ang2 =  np.arccos(((h*h)+ (suct_gripper*suct_gripper)+(2*suct_gripper*l))/(2*(l+suct_gripper)*h))
ang3 = np.pi - ang1 - ang2
	

theta1 = ang3 -np.pi/2
	
thetas = [theta0, theta1] # angles are in radians
return(thetas)