#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
import math

class JointCmds: # dictionary mapping to joint position values
    def __init__(self, num_modules):
        self.num_modules = num_modules
        self.jnt_cmd_dict = {}
        self.joints_list = [] # list of name of modules e.g. "SA_001"
        self.make_joint_list()
    
    def make_joint_list(self): # creates list of module names
        for i in range(self.num_modules):
            leg_str='S_'
            if i < 10 :
                leg_str += '0' + str(i)
            else :
                leg_str += str(i)
            self.joints_list += [leg_str]
    
    def get_next_cmd(self):
        return self.jnt_cmd_dict

class SnakeControl:
    def __init__(self, num_modules, hz, dt):
        self.num_modules = num_modules
        self.hz = hz
        self.t = 0.0
        self.dt = dt
        self.isPaused = False
        self.sensor_efforts = []
        self.sensor_velocities = []
        rospy.Subscriber("/snake/joint_states", JointState,
                            self.call_joint_efforts)
        rospy.Subscriber("/snake/joint_states", JointState,
                            self.call_joint_velocities)
        
        pub = {} # one publisher per joint
        ns_str = '/snake'
        cont_str = 'eff_pos_controller'
        for i in range(num_modules) :
            leg_str='S_'
            if i < 10 :
                leg_str += '0' + str(i)
            else :
                leg_str += str(i)
            pub[leg_str] = rospy.Publisher(ns_str + '/' + leg_str + '_'
                                            + cont_str + '/command',
                                            Float64, queue_size=10)
        rate = rospy.Rate(self.hz)
        self.curr_cmds = JointCmds(num_modules=self.num_modules) # command robot should currently be doing
        self.next_cmds = JointCmds(num_modules=self.num_modules) # command robot should do next
        while not rospy.is_shutdown():
            next_command = self.curr_cmds.get_next_cmd()
            for joint in self.curr_cmds.jnt_cmd_dict.keys():
                pub[joint].publish(next_command[joint])
            if (not self.isPaused):
                self.t += dt # keep track of time
                self.make_gait()
                # MORE STUFF HERE TO LOOP
                # self.call_efforts()
                # self.call_velocity()
                # self.call_IMU()
            rate.sleep()

    def make_gait(self):
        # self.wraparound()
        for i, joint in enumerate(self.next_cmds.joints_list):
            self.next_cmds.jnt_cmd_dict[joint] = self.helix_climb(i)
            print str(i) + " angle: " + str(self.next_cmds.jnt_cmd_dict[joint])
        self.curr_cmds.jnt_cmd_dict = self.next_cmds.jnt_cmd_dict # set command
    
    def helix_climb(self, i):
        m = 0.4 # length of module
        r = 1.0 # radius
        p = 1.5 # pitch
        # s = i * m # arc length
        k = r/(r**2 + p**2) # curvature
        t = p/(r**2 + p**2) # torsion
        # k_cos = k*math.cos(t*s)
        # k_sin = k*math.sin(t*s)
        A = (2*k/t)*math.sin(t*m)
        alpha_cos = 5*A*math.cos(3*self.t + t*m*i) # m = length of module; i = index of joint
        alpha_sin = 5*A*math.sin(3*self.t + t*m*i)
        if (i%4 == 1 or i%4 == 2):
            alpha_cos *= -1
            alpha_sin *= -1
        if (i%2 == 0):
            return alpha_cos
        else:
            return alpha_sin
    
    def rolling(self, i):
        m = 0.05 # length of module
        r = 0.8 # radius
        p = 0.2 # pitch
        A = (2*m/r)
        alpha_cos = A*math.cos(5*self.t) # m = length of module; i = index of joint
        alpha_sin = A*math.sin(5*self.t)
        if (i%4 == 1 or i%4 == 2):
            alpha_cos *= -1
            alpha_sin *= -1
        if (i%2 == 1): ## SET REMAINDER TO 1 OR 0 TO FLIP ORIENTATION
            return alpha_cos
        else:
            return alpha_sin

    def wraparound(self): # this wraps snake around a pole
        average_effort = 0
        for effort in self.sensor_efforts:
            average_effort += abs(effort)
        average_effort /= self.num_modules
        print(average_effort)
        self.curl_to_size(0.4)
        for i, joint in enumerate(self.next_cmds.joints_list):
            if (i >= 8):
                if (i%2 == 1):
                    self.next_cmds.jnt_cmd_dict[joint] = np.pi/4
                    if ((i+1)%4 == 0):
                        self.next_cmds.jnt_cmd_dict[joint] *= -1
        


    def curl_to_size(self, size):
        for i, joint in enumerate(self.next_cmds.joints_list):
            if (i%2 == 0):
                self.next_cmds.jnt_cmd_dict[joint] = np.pi/(8 * size)
                if (i%4 == 0):
                    self.next_cmds.jnt_cmd_dict[joint] *= -1
    
    def call_joint_efforts(self, data):
        self.sensor_efforts = list(data.effort)
    
    def call_joint_velocities(self, data):
        self.sensor_velocities = list(data.velocity)


if __name__ == '__main__':
    rospy.init_node("main_control")
    try:
        snake_control = SnakeControl(num_modules=16, hz=100, dt=1.0/100)
    except rospy.ROSInterruptException:  pass