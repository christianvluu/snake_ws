#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import copy
from noise import *
from data_processing import *


IS_COMPLIANT = True # run compliant algorithm?
FILE_LOC = '/home/christianluu/snake_ws/data/'
FILE_NAME = "sensorefforts_data_r0.060_k1.3_amp_2.1"
FILE_EFFORTS = FILE_LOC + FILE_NAME + "_efforts.txt"
FILE_CURRENTS = FILE_LOC + FILE_NAME + "_currents.txt"
FILE_THETAS = FILE_LOC + FILE_NAME + "_thetas.txt"
RECORD_DATA = False # to record all data, IS_CURRENT also needs to be True
USE_MODEL = True
IS_CURRENTS = True
COMPLIANCE_WITH_CURRENTS = False

RECORD_POS = False # record height data for graphs
FILE_POS = FILE_LOC + FILE_NAME + "_pos.txt"

"""
MODES TO RUN THIS FILE USING FLAGS
1) Normal with(out) Compliance (don't use currents, no logging)
   - IS_COMPLIANCE = True/False
   - RECORD_DATA = False
   - USE_MODEL = False
   - IS_CURRENTS = False
   - COMPLIANCE_WITH_CURRENTS = False
   - RECORD_POS = False
2) Normal with(out) Compliance (USE currents, no logging)
   - IS_COMPLIANCE = True/False
   - RECORD_DATA = False
   - USE_MODEL = False
   - IS_CURRENTS = True
   - COMPLIANCE_WITH_CURRENTS = True
   - RECORD_POS = False
3) Datalogging with(out) Compliance (don't currents, log efforts, theta, pos, simulated currents)
   - IS_COMPLIANCE = True/False
   - RECORD_DATA = True
   - USE_MODEL = False
   - IS_CURRENTS = True
   - COMPLIANCE_WITH_CURRENTS = False
   - RECORD_POS = True
4) Compliance with Model (don't use currents, log efforst, theta, pos, simulated currents)
   - IS_COMPLIANCE = True
   - RECORD_DATA = False
   - USE_MODEL = True
   - IS_CURRENTS = True
   - COMPLIANCE_WITH_CURRENTS = False
   - RECORD_POS = True
   

#### NOT VERY USEFUL BELOW ####
3) Datalogging with(out) Compliance (don't use currents, log efforts, theta, pos, simulated currents)
   - IS_COMPLIANCE = True/False
   - RECORD_DATA = True
   - USE_MODEL = False
   - IS_CURRENTS = True
   - RECORD_POS = True
"""

class SnakeControl:
    def __init__(self, num_modules, hz, dt):
        self.num_modules = num_modules
        self.hz = hz
        self.t = 0.0
        self.dt = dt
        self.isPaused = False

        # use this for currentSpikeGenerator
        self.sensor_efforts = np.zeros(num_modules)
        self.simulated_currents = np.zeros(num_modules)
        self.prev_cmd_theta = np.zeros(num_modules)
        self.curr_cmd_theta = np.zeros(num_modules)
        self.prev2curr_delta = np.zeros(num_modules)
        self.spike_loop = np.zeros(num_modules)

        # reporting pose status
        self.pos_z = np.zeros(self.num_modules)

        self.sim_currents = np.zeros(num_modules)
        self.sensor_velocities = np.zeros(num_modules)
        self.sensor_linear_accel_x = np.zeros(num_modules)
        self.sensor_linear_accel_y = np.zeros(num_modules)
        self.sensor_linear_accel_z = np.zeros(num_modules)
        self.sensor_angular_vel_x = np.zeros(num_modules)
        self.sensor_angular_vel_y = np.zeros(num_modules)
        self.sensor_angular_vel_z = np.zeros(num_modules)
        self.sensor_orientation_x = np.zeros(num_modules)
        self.sensor_orientation_y = np.zeros(num_modules)
        self.sensor_orientation_z = np.zeros(num_modules)
        rospy.Subscriber("/snake/joint_states", JointState,
                            self.call_joint_efforts)
        rospy.Subscriber("/snake/joint_states", JointState,
                            self.call_joint_velocities)
        rospy.Subscriber("/snake/sensors/SA001__MoJo/imu", Imu,
                            self.call_imu_001)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                            self.call_pose)
        self.state = "roll_to_pole"

        self.p = 0 # pitch
        self.r = 0 # radius
        self.state_change_time = 0.0

        self.vel = 0
        self.A = 0.3 # amplitude used for rolling

        self.const = {
            "l": 0.07,
            "Md": 0.1,
            "Bd": 2,
            "Kd": 1.5,
            "k": 1.3, # 1.3 is good shape value; ALTER THIS to change how the snake wraps around pole
            "target_amp": 2.1, # 1.8 is good; max for NO COMPLIANCE is 1.55
            "w_t": 4 # speed of rolling
        }
        if (RECORD_DATA): 
            self.fT = open(FILE_THETAS,"a+") # fT = FILE_THETAS
            self.fC = open(FILE_CURRENTS, "a+")
            self.fE = open(FILE_EFFORTS, "a+")

            self.fT.write("0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15\n")
            self.fE.write("0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15\n")
            self.fC.write("0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15\n")
            # self.fT.write("l,Md,Bd,Kd,k,amp,w_t\n")
            # self.fT.write(str(self.const["l"]) + "," + str(self.const["Md"]) + "," + str(self.const["Bd"]) + "," + str(self.const["Kd"]) + "," + str(self.const["k"]) + "," + str(self.const["target_amp"]) + "," +  str(self.const["w_t"]) + "\n")
            # self.fE.write("l,Md,Bd,Kd,k,amp,w_t\n")
            # self.fE.write(str(self.const["l"]) + "," + str(self.const["Md"]) + "," + str(self.const["Bd"]) + "," + str(self.const["Kd"]) + "," + str(self.const["k"]) + "," + str(self.const["target_amp"]) + "," +  str(self.const["w_t"]) + "\n")
            # self.fC.write("l,Md,Bd,Kd,k,amp,w_t\n")
            # self.fC.write(str(self.const["l"]) + "," + str(self.const["Md"]) + "," + str(self.const["Bd"]) + "," + str(self.const["Kd"]) + "," + str(self.const["k"]) + "," + str(self.const["target_amp"]) + "," +  str(self.const["w_t"]) + "\n")

        if (RECORD_POS):
            self.fP = open(FILE_POS, "a+")
            self.fP.write("time,index,height\n")
            # self.fP.write("l,Md,Bd,Kd,k,amp,w_t\n")
            # self.fP.write(str(self.const["l"]) + "," + str(self.const["Md"]) + "," + str(self.const["Bd"]) + "," + str(self.const["Kd"]) + "," + str(self.const["k"]) + "," + str(self.const["target_amp"]) + "," +  str(self.const["w_t"]) + "\n")

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
    
    def gait_caller(self): # function to assign commands, gets called once per increment of self.dt
        next_theta = self.gait_generator()
        if (RECORD_DATA):
            # self.fT.write(str(self.t) + " Theta:")
            for i in range(0, len(next_theta)):
                if (i != len(next_theta)-1):
                    self.fT.write(str(next_theta[i]) + ",")
                else:
                    self.fT.write(str(next_theta[i]) + "\n")
        for i, joint in enumerate(self.next_cmds.joints_list):
            self.prev_cmd_theta[i] = copy.deepcopy(self.curr_cmd_theta[i])
            self.curr_cmd_theta[i] = next_theta[i]
            self.next_cmds.jnt_cmd_dict[joint] = next_theta[i]
            # setting the actual commands
        self.curr_cmds.jnt_cmd_dict = self.next_cmds.jnt_cmd_dict
        
        
        if (RECORD_POS):
            print("pos Z: ", self.pos_z[2])
            self.fP.write(str(self.t) + " " + "0 " + str(self.pos_z[2]) + "\n")

        #print(np.max(next_theta))
    
    def gait_generator(self):
        if (USE_MODEL):
            # temp = changeSEAToUnified(self.sensor_efforts)
            efforts_predicted = np.zeros(self.num_modules)
            for i in range(0, self.num_modules):
                # use the sklearn linear predictor computed in data_processing.py to calculate
                efforts_predicted[i] = regressor.predict(np.array([[self.simulated_currents[i]]]))[0][0]
                print("Prediction: ", efforts_predicted[i], " vs Real: ", self.sensor_efforts[i])
            efforts_unified = changeSEAToUnified(efforts_predicted)
        elif (IS_CURRENTS and COMPLIANCE_WITH_CURRENTS):
            efforts_unified = changeSEAToUnified(self.simulated_currents)
        else: 
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
        # print("raw")
        #print(self.sensor_efforts[1:5])

        if (RECORD_DATA):
            for i in range(0, len(self.sensor_efforts)):
                if (i != len(self.sensor_efforts)-1):
                    self.fE.write(str(self.sensor_efforts[i]) + ",")
                else:
                    self.fE.write(str(self.sensor_efforts[i]) + "\n")

        if (IS_CURRENTS):
            # make array for simulated_currents
            for i in range(0, len(self.sensor_efforts)):
                self.simulated_currents[i] = copy.deepcopy(self.sensor_efforts[i])

            
            # add white noise
            self.simulated_currents += add_white_gaussian_noise(self.sensor_efforts)
            #print("white noise:")
            #print(self.sensor_efforts[1:5])
            
            # if (RECORD_DATA):
            #     self.f.write(str(self.t) + " AWGN: ")
            #     for i in range(0, len(self.sensor_efforts)):
            #         self.f.write(" " + str(self.sensor_efforts[i]))
            #     self.f.write("\n")

            # add low freq  noise
            self.simulated_currents += add_low_freq_noise(self.sensor_efforts, self.t)
            #print("low freq noise:")
            #print(self.sensor_efforts[1:5])

            # if (RECORD_DATA):
            #     self.f.write(str(self.t) + " LowFreq: ")
            #     for i in range(0, len(self.sensor_efforts)):
            #         self.f.write(" " + str(self.sensor_efforts[i]))
            #     self.f.write("\n")

            # add curr_spike
            for i in range(0, len(self.sensor_efforts)):
                self.simulated_currents[i] *= add_current_spike(self.prev_cmd_theta[i], self.curr_cmd_theta[i], self.spike_loop, self.prev2curr_delta, i)
            #print("spike generator:")
            # print(self.prev2curr_delta)
            #print(self.sensor_efforts[1:5])

            if (RECORD_DATA):
                # self.f.write(str(self.t) + "")
                for i in range(0, len(self.simulated_currents)):
                    if (i != len(self.sensor_efforts)-1):
                        self.fC.write(str(self.simulated_currents[i]) + ",")
                    else:
                        self.fC.write(str(self.simulated_currents[i]) + "\n")
                # self.fC.write("\n")
        
        return True
    
    def call_joint_velocities(self, data):
        self.sensor_velocities = list(data.velocity)
        # print("vel: ", self.sensor_velocities[5])
        return True
    
    def call_pose(self, data):
        # self.pos_z = np.zeros(self.num_modules)
        for i, pose in enumerate(data.pose):
            if (pose.position.z != 0):
                self.pos_z[i] = pose.position.z
            #print("posz: ", pose.position.z, "i: ", i)
        # writing POS data to .txt is in gait_caller function

    def call_imu_001(self, data):
        i = 0
        self.sensor_linear_accel_x[i] = data.linear_acceleration.x
        self.sensor_linear_accel_y[i] = data.linear_acceleration.y
        self.sensor_linear_accel_z[i] = data.linear_acceleration.z
        self.sensor_angular_vel_x[i] = data.angular_velocity.x
        self.sensor_angular_vel_y[i] = data.angular_velocity.y
        self.sensor_angular_vel_z[i] = data.angular_velocity.z
        self.sensor_orientation_x[i] = data.orientation.x
        self.sensor_orientation_y[i] = data.orientation.y
        self.sensor_orientation_z[i] = data.orientation.z
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
    w_t = const["w_t"] # temporal frequency, curve of snake backbone (circular)
    
    theta = np.zeros(num_modules)
    for i in range(0, num_modules):
        if (i%2 == 0): # theta dorsal
            theta[i] = amp*math.sin(w_s*i*l - w_t*time)
        else:  # theta lat
            theta[i] = amp*math.sin(w_s*i*l - w_t*time + math.pi/2)

    return theta

def generate_serpernoid_curve_derivative(i, amp, k, time, l, w_t):
    if (i % 2 == 0): # d(theta)/d(amp)
        return math.sin(amp*k*i*l - w_t*time) + amp*math.cos(amp*k*i*l - w_t*time)*(k*i*l)
    else:
        return math.sin(amp*k*i*l - w_t*time + math.pi/2) + amp*math.cos(amp*k*i*l - w_t*time + math.pi/2)*(k*i*l)



def generate_shape_parameter(amp, vel, efforts, time, dt, const): # efforts are alr unified
    l = const["l"]
    Md = const["Md"]
    Bd = const["Bd"]
    Kd = const["Kd"]
    k = const["k"]
    target_amp = const["target_amp"]
    w_s = amp * k # spatial frequency, for the curve to helix
    w_t = const["w_t"] # temporal frequency, curve of snake backbone (circular)
    num_modules = efforts.shape[0]

    # FAKE EFFORT values!!!!!!!!!!!!!!!!!!!!!!!
    #efforts = fakeEffortGenerator(time, num_modules)

    J = np.zeros(num_modules) # jacobian to map shape forces
    for i in range(0, num_modules):
        # derivative of serpenoid curve wrt to shape parameter (amplitude)
        # J[i] = generate_serpernoid_curve_derivative(i, amp, k, time, l, w_t)
        if (i%2 == 0):
            J[i - 1] = math.sin(w_s*i*l - w_t*time)
        else:
            J[i - 1] = math.sin(w_s*i*l - w_t*time + math.pi/2)

        

    tau_J = 0.2*np.matmul(J, efforts) # this should be a single value, applied effort to pole
    vel = (tau_J - Bd*vel - Kd*(amp-target_amp))*(dt/Md) + vel
    new_amp = vel*dt + amp
    # print "time:", time, "amplitude:", new_amp, " vel:", vel, "tau_J:", tau_J, "efforts:", efforts
    print "time: ", time, "shape parameter:", amp
    # print "efforts: ", efforts
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

def whiteNoiseGenerator(efforts, num_modules, currents): # adds AWGN noise
    
    # Additive White Gaussian Noise (AWGN)
    if (efforts[0] != 0): # ensure the effort values from the sim is != 0
        SNR_db = 20 # signal to noise ratio (dB)
        sig_avg_efforts = np.mean(np.abs(efforts))
        sig_avg_db = 10 * np.log10(sig_avg_efforts)

        noise_avg_db = sig_avg_db - SNR_db
        noise_avg_effort = 10 ** (noise_avg_db/10)
        noise_efforts = np.random.normal(0, np.sqrt(noise_avg_effort), num_modules)
        currents = noise_efforts + efforts

def currentSpikeGenerator(num_modules, currents, prev_pos, curr_pos, delta):
    # delta is the difference between prev_pos and curr_pos (curr_pos - prev_pos)
    # during the previous command sent to snake (used to keep track of direction of change)
    for i in range(0, num_modules):
        curr_delta = curr_pos - prev_pos
        if (checkSign(delta[i]) !=  checkSign(curr_delta)): # add current spike if not in same direction
            currents[i] *= 1.5 # the abs(i-7)/14 part is to scale current spike, higher current spike in the middle modules
        delta[i] = curr_delta


def checkSign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    else:
        return 0



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