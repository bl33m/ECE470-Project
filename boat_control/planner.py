import numpy as np
from simple_pid import PID # >>pip3 install simple-pid
import matplotlib.pyplot as plt
import math
from scipy.spatial.transform import Rotation as R # >>pip3 install scipy
from collections import deque

# PID further info
# https://github.com/m-lundberg/simple-pid

def pi_clip(angle):
    '''Function to map angle error values between [-pi, pi)'''
    if angle > 0:
        if angle > math.pi:
            return angle - 2*math.pi
    else:
        if angle < -math.pi:
            return angle + 2*math.pi
    return angle

def transformations(Rab, Rbc, Tab, Tbc):
    '''
    Arguments:
        Rab: Rotation matrix from coordinate reference frame b to reference frame a (numpy.ndarray (3,3))
        Rbc: Rotation matrix from coordinate reference frame c to reference frame b (numpy.ndarray (3,3))
        Tab: Translation of b with respect to a (numpy.ndarray(3,))
        Tbc: Translation of c with respect to b (numpy.ndarray(3,))
    Return:
        Rac: Rotation matrix from coordinate reference frame c to reference frame a (numpy.ndarray (3,3))
        quat_ac: quaternion (in order: qx, qy, qz, qw) from coordinate frame c to a (numpy.ndarray (4,))
        euler_ac: Euler angles (in rads and 'xyz' order) from reference frame c to a (numpy.ndarray(3,))
        Tac: Translation of c with respect to a (numpy.ndarray(3,))
    '''
    Rac = Rab@Rbc # ... your code here
    r = R.from_matrix(Rac)
    quat_ac = r.as_quat()  # ... 
    euler_ac = r.as_euler('xyz') # ... 
    Tac = Rab@Tbc + Tab # ... 

    return Rac, quat_ac, euler_ac, Tac


class path_planner:

    pid_w = PID(-1, 0, 0, setpoint=0.0, output_limits=(-5, 5))
    pid_v = PID(-1, 0, 0, setpoint=0.0, output_limits=(0, 2))
    pid_w.error_map = pi_clip #Function to map angle error values between -pi and pi.
    
    def __init__(self):
        
        self.x = 0.0 # (x,y) Robot's position
        self.y = 0.0
        self.xd = 15.0 # (xd, yd) is the desired goal
        self.yd = 15.0
        self.key_points = deque()
        self.turn_rad = 5
        self.time = np.arange(0,40,0.1)
        self.dt = 0.1
        
        self.v = 0 # Forward velocity
        self.w = 0 # Angular velocity
        self.theta = 0 # Heading angle
        self.results = [[],[],[],[],[],[],[]] 
        self.turning = False
        self.plan_path()
        self.xd = self.key_points.popleft()[0]
        self.yd = self.key_points.popleft()[1]
        for t in self.time: # control loop
            self.desired_trajectory(t)
            self.update_robot_state()
            angle_error, distance_error = self.compute_error()
            self.w, self.v = self.compute_control(angle_error, distance_error)
            print(self.x, self.y)
            self.save_results()

    def plan_path(self):
        circle_x = self.xd - self.turn_rad
        circle_y = self.yd - self.turn_rad
        self.key_points.append([circle_x - self.turn_rad, self.yd])
        self.key_points.append([circle_x - self.turn_rad*np.cos(np.pi/4),  circle_y + self.turn_rad*np.sin(np.pi/4)])
        self.key_points.append([circle_x, circle_y + self.turn_rad])
        self.key_points.append([circle_x + self.turn_rad*np.cos(np.pi/4),  circle_y + self.turn_rad*np.sin(np.pi/4)])
        self.key_points.append([self.xd, self.yd])
        print(self.key_points)

    def desired_trajectory(self, t):
        angle_error, distance_error = self.compute_error()
        print(distance_error)
        """
        if t < 10:
            self.xd = self.key_points[0][0]
            self.yd = self.key_points[0][1]
        if t > 10:
            self.xd = self.turn_rad*np.cos(2*np.pi*0.03*(t-30)) + self.key_points[2][0] - self.turn_rad
            self.yd = self.turn_rad*np.sin(2*np.pi*0.03*(t-30)) + self.key_points[2][1] - self.turn_rad
        """ 
        if len(self.key_points) > 1:
            if distance_error < 0.1:
                self.xd = self.key_points.popleft()[0]
                self.yd = self.key_points.popleft()[1]
    
    def update_robot_state(self):
        self.x += self.v*np.cos(self.theta)*self.dt # ...
        self.y += self.v*np.sin(self.theta)*self.dt # ...
        self.theta += self.w*self.dt # ...
        
    
    def compute_error(self):
        rho = np.sqrt((self.xd - self.x)**2 + (self.yd - self.y)**2) 
        distance_error = rho # ...
        angle_error = np.arctan2((self.yd - self.y), (self.xd - self.x)) - self.theta # ...
        return angle_error, distance_error
    
    def compute_control(self, angle_error, distance_error): # It computes the control commands
        control_w = self.pid_w(angle_error, dt = self.dt)
        control_v = self.pid_v(distance_error, dt = self.dt)
        return control_w, control_v

    def save_results(self):
        self.results[0].append(self.x)
        self.results[1].append(self.y)
        self.results[5].append(self.w)
        self.results[6].append(self.v)


if __name__ == '__main__':
    Rab = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    Rbc = np.array([[1,0,0],[0,0,1],[0,-1,0]])
    Tab = np.array([1,2,3])
    Tbc = np.array([4,5,6])
    Rac, quat, euler, Tac = transformations(Rab, Rbc,Tab, Tbc)

    planner = path_planner()
    plt.figure (1)
    plt.plot(planner.results[0], planner.results[1],'b')
    plt.xlabel("x")
    plt.ylabel("y")

    plt.figure (2)
    plt.plot(planner.results[5],'b')
    plt.plot(planner.results[6],'r')
    plt.legend(["Angular velocity w", "Forward velocity v"])
    plt.show()
