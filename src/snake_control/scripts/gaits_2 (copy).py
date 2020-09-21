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
        self.sensor_efforts = [([0] * 1) for i in range(0, num_modules)]
        self.smoothed_sensor_efforts = [0] * num_modules
        self.sensor_velocities = [0] * num_modules
        rospy.Subscriber("/snake/joint_states", JointState,
                            self.call_joint_efforts)
        rospy.Subscriber("/snake/joint_states", JointState,
                            self.call_joint_velocities)
        self.state = "roll_to_pole"

        self.p = 0 # pitch
        self.r = 0 # radius
        self.state_change_time = 0.0

        self.vel = 0
        self.A = 0.18
        
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
                #print(np.average(self.sensor_efforts))
                # self.call_IMU()
            rate.sleep()

    def make_gait(self): # loop
        # self.wraparound()
        for i, joint in enumerate(self.next_cmds.joints_list):
            self.next_cmds.jnt_cmd_dict[joint] = self.helix_climb_ruscelli(i)
            #print str(i) + " angle: " + str(self.next_cmds.jnt_cmd_dict[joint])
        self.compliance()
        self.curr_cmds.jnt_cmd_dict = self.next_cmds.jnt_cmd_dict # set command

    def compliance(self): # this provides amplitude (A) values to 
        l = 0.07 # length of module
        Kd = 1 # 5
        Md = 1 # suggested 0.1
        Bd = 5  # 2
        A = 1.7 # was previously 1.2
        k = 2.5
        w_s = self.A * k # spatial frequency, for the curve to helix
        w_t = w_s # 2 # temporal frequency, curve of snake backbone (circular)

        J = np.zeros(self.num_modules)
        for i in range(1, self.num_modules+1): # NEED TO FLIP EVERY 3rd/4th JOINT
            # if i%2:
            #     ### DO DORSAL STUFF
            # else:
            #     ### DO 
            J[i - 1] = math.sin(w_s*i*l + w_t*self.t) 
        #J = J.reshape(-1, 1)
        #J = J.transpose()
        # self.smoothed_sensor_efforts = np.array([0.5]*self.num_modules) # TESTING FAKE tau_ext values
        ##TESTING ADD BACK ###effort_reshaped = np.array(self.smoothed_sensor_efforts)
        # effort_0 = np.full((self.num_modules, 1), 0.37).reshape(-1, 1) # calibration efforts, this is basically "0"
        
        ##TESTING ADD BACK####tau_0 = np.matmul(J, effort_reshaped)
        
        ## TESTING ADD BACK#####tau = np.matmul(J, effort_reshaped)
        tau = np.matmul(J, self.sensor_efforts) ## TESTING
        # maybe add LPF for tau

        # spring mass damper system
        self.vel = (tau - Bd*self.vel - Kd*(self.A-A))*(self.dt/Md) + self.vel

        self.A = self.vel*self.dt + self.A

    
    def helix_climb_ruscelli(self, i): # DEPRECATED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        k = 1.5      #b
        A_lat = 0.18 * (self.t)*0.18 # amplitude
        A_lat = self.A
        print(self.t, A_lat)
        A_dor = A_lat
        w_s_lat = A_lat * k # spatial frequency, for the curve to helix
        w_s_dor = w_s_lat
        w_t_lat = w_s_lat # temporal frequency, curve of snake backbone (circular)
        w_t_dor = w_t_lat
        l = 0.07 # length of module
        s_i = i * l # distances from head of module for module i

        # theta_lat = -A_lat*math.sin(w_s_lat*s_i + w_t_lat*self.t) # Don't use this, the mix of sin and cosine
        # theta_dor = A_dor*math.cos(w_s_dor*s_i + w_t_dor*self.t)  # makes this confusing, use below instead

        theta_lat = A_lat*math.sin(w_s_lat*s_i + w_t_lat*self.t + math.pi/2)
        theta_dor = A_dor*math.sin(w_s_dor*s_i + w_t_dor*self.t )

        if (i%4 == 1 or i%4 == 2):
            theta_dor *= -1
            theta_lat *= -1
        

        if (i%2 == 0):
            return theta_dor
        else:
            return theta_lat
    

    def call_joint_efforts(self, data):
        for i, effort in enumerate(data.effort):
            #effort = abs(effort)
            #self.sensor_efforts[i].append(effort)
            self.sensor_efforts[i] = effort
            #self.smoothed_sensor_efforts[i] = running_average_filter(effort, self.sensor_efforts[i], i)
        #self.sensor_efforts = list(data.effort)
        #print(self.smoothed_sensor_efforts)
        return True
    
    def call_joint_velocities(self, data):
        self.sensor_velocities = list(data.velocity)
        return True

def running_average_filter(new_val, val_list, i, running_count=10):
    if len(val_list) > running_count and new_val > 0:
        val_list.pop(1) # remove the front one
    return np.average(val_list)



if __name__ == '__main__':
    rospy.init_node("main_control")
    try:
        snake_control = SnakeControl(num_modules=16, hz=100, dt=1.0/100)
    except rospy.ROSInterruptException:  pass