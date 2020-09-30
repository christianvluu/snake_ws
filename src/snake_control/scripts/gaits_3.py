#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
import math


IS_COMPLIANT = True # run compliant algorithm?

class SnakeControl:
    def __init__(self, num_modules, hz, dt):
        self.num_modules = num_modules
        self.hz = hz
        self.t = 0.0
        self.dt = dt
        self.isPaused = False
        self.sensor_efforts = np.zeros(num_modules)
        self.smoothed_sensor_efforts = np.zeros(num_modules)
        self.sensor_velocities = np.zeros(num_modules)
        rospy.Subscriber("/snake/joint_states", JointState,
                            self.call_joint_efforts)
        rospy.Subscriber("/snake/joint_states", JointState,
                            self.call_joint_velocities)
        self.state = "roll_to_pole"

        self.p = 0 # pitch
        self.r = 0 # radius
        self.state_change_time = 0.0

        self.vel = 0
        self.A = 0.3 # amplitude used for rolling

        self.const = {
            "l": 0.07,
            "Md": 0.5,
            "Bd": 1,
            "Kd": 7,
            "k": 1.25, # 1.3 is good shape value; ALTER THIS to change how the snake wraps around pole
            "target_amp": 1.85 # 1.8 is good; max for NO COMPLIANCE is 1.55
        }
        
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
        self.curr_cmds = JointCmds(num_modules=self.num_modules)
        self.next_cmds = JointCmds(num_modules=self.num_modules)
        while not rospy.is_shutdown():
            next_command = self.curr_cmds.get_next_cmd()
            for joint in self.curr_cmds.jnt_cmd_dict.keys():
                pub[joint].publish(next_command[joint])
            if (not self.isPaused):
                self.t += dt # keep track of time
                self.gait_caller()
                # MORE STUFF HERE TO LOOP
                #print(np.average(self.sensor_efforts))
                # self.call_IMU()
            rate.sleep()
    
    def gait_caller(self): # function to assign commands
        next_theta = self.gait_generator()
        for i, joint in enumerate(self.next_cmds.joints_list):
            self.next_cmds.jnt_cmd_dict[joint] = next_theta[i]
            # setting the actual commands
        self.curr_cmds.jnt_cmd_dict = self.next_cmds.jnt_cmd_dict
        #print(np.max(next_theta))
    
    def gait_generator(self):
        efforts_unified = changeSEAToUnified(self.sensor_efforts)
        if (IS_COMPLIANT):
            self.A = generate_shape_parameter(self.A, self.vel, efforts_unified,
                                            self.t, self.dt, self.const)
        else:
            self.A = self.const["target_amp"]
        theta = generate_serpernoid_curve(self.A, self.t, self.const,
                                       self.num_modules)
        theta_sea = changeUnifiedToSEA(theta)
        return theta_sea
    
    def call_joint_efforts(self, data): # gets called automatically by subcriber
        for i, effort in enumerate(data.effort):
            if (effort != 0):
                self.sensor_efforts[i] = effort
            #print(self.sensor_efforts)
        return True
    
    def call_joint_velocities(self, data):
        self.sensor_velocities = list(data.velocity)
        return True



# generates unified thetas
def generate_serpernoid_curve(amp, time, const, num_modules):
    l = const["l"]
    Md = const["Md"]
    Bd = const["Bd"]
    Kd = const["Kd"]
    k = const["k"]
    target_amp = const["target_amp"]
    w_s = amp * k # spatial frequency, for the curve to helix
    w_t = w_s # temporal frequency, curve of snake backbone (circular)
    
    theta = np.zeros(num_modules)
    for i in range(0, num_modules):
        if (i%2 == 0): # theta dorsal
            theta[i] = amp*math.sin(w_s*i*l - w_t*time)
        else:  # theta lat
            theta[i] = amp*math.sin(w_s*i*l - w_t*time + math.pi/2)

    return theta

def generate_serpernoid_curve_derivative(i):
    if (i % 2 == 0):
        return math.sin(amp*k*l - amp*k*time) + a*cos(amp*k*l - a*k*t)*k*l
    else:
        return math.sin(amp*k*l - amp*k*time + math.pi/2) + a*cos(amp*k*l - a*k*t + math.pi/2)*k*l



def generate_shape_parameter(amp, vel, efforts, time, dt, const): # efforts are alr unified
    l = const["l"]
    Md = const["Md"]
    Bd = const["Bd"]
    Kd = const["Kd"]
    k = const["k"]
    target_amp = const["target_amp"]
    w_s = amp * k # spatial frequency, for the curve to helix
    w_t = w_s # temporal frequency, curve of snake backbone (circular)
    num_modules = efforts.shape[0]

    # FAKE EFFORT values!!!!!!!!!!!!!!!!!!!!!!!
    #efforts = fakeEffortGenerator(time, num_modules)

    J = np.zeros(num_modules) # jacobian to map shape forces
    for i in range(1, num_modules + 1):
        # derivative of serpenoid curve wrt to shape parameter (amplitude)
        if (i%2 == 0):
            J[i - 1] = math.sin(w_s*i*l - w_t*time)
        else:
            J[i - 1] = math.sin(w_s*i*l - w_t*time + math.pi/2)

        

    tau_J = 0.1*np.matmul(J, efforts) # this should be a single value, applied effort to pole
    vel = (tau_J - Bd*vel - Kd*(amp-target_amp))*(dt/Md) + vel
    new_amp = vel*dt + amp
    #print "time:", time, "amplitude:", new_amp, " vel:", vel, "tau_J:", tau_J, "efforts:", efforts
    print "time: ", time, "shape parameter:", amp
    return new_amp

def fakeEffortGenerator(time, num_modules):
    efforts = np.zeros(num_modules)
    ## GENERATE FAKE effort values
    for i in range(0, num_modules):
        if (time < 18): # sl0.1owly increase effort values up to 0.6 until time = 18
            efforts[i] = 4 #time/30
        elif (time >= 18 and time <= 25): # keep constant effort from 18-25sec
            efforts[i] = 4 #0.6
        else: # return effort = 0.1 after 25 sec
            efforts[i] = 4 #0.1
    return efforts


def changeSEAToUnified(SEA): # change from real snake orientation to unified for algorithm
    num_modules = SEA.shape[0]
    unified = np.zeros(num_modules)
    for i in range(0, num_modules):
        if (i % 4 > 1):
            unified[i] = -SEA[i]
        else:
            unified[i] = SEA[i]
    return unified

def changeUnifiedToSEA(unified):
    return changeSEAToUnified(unified)

def running_average_filter(new_val, val_list, i, running_count=10):
    if len(val_list) > running_count and new_val != 0:
        val_list.pop(1) # remove the front one
    return np.average(val_list)

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


if __name__ == '__main__':
    rospy.init_node("main_control")
    try:
        snake_control = SnakeControl(num_modules=16, hz=100, dt=1.0/100)
    except rospy.ROSInterruptException:  pass