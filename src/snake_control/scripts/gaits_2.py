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

        self.vel = [[0]]
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

    def helix_climb_ruscelli_with_compliance(self, i): # from Ruscelli compliance paper
        vel = 0

        ##### NEED TO REWRITE CODE SO THAT ENTIRE MATRIX/ARRAY OF THETA OF SNAKE IS CALCULATED AT ONCE, NOT WHEN WE INPUT IN THIS FUNCITON i
        k = 3 # 10 seems to work       
        A_lat = 0.18 # amplitude
        A_dor = A_lat
        w_s_lat = A_lat * k # spatial frequency, for the curve to helix
        w_s_dor = w_s_lat
        w_t_lat = 2 # temporal frequency, curve of snake backbone (circular)
        w_t_dor = w_t_lat
        l = 0.07 # length of module
        s_i = i * l # distances from head of module for module i

        k = w_s_lat/A_lat # factor linking amplitude (A) and spatial freq (w_s)

        J = np.zeros(self.num_modules)
        for i1 in range(1, self.num_modules+1):
            J[i1 - 1] = A_lat*math.sin(w_s_lat*i1*l + w_t_lat*self.t)
        J = J.reshape(-1, 1)
        J = J.transpose()

        effort_reshaped = np.array(self.smoothed_sensor_efforts).reshape(-1, 1)
        effort_0 = np.zeros(self.num_modules).reshape(-1, 1)

        tau_0 = np.matmul(J, effort_reshaped)
        tau = abs(np.matmul(J, effort_reshaped))

        A_calculated = (tau - Bd*vel - Kd*(L-L0))*(dt/Md) + vel

        theta_lat = -A_lat*math.sin(w_s_lat*s_i + w_t_lat*self.t)
        theta_dor = A_dor*math.cos(w_s_dor*s_i + w_t_dor*self.t)

    def compliance(self): # this provides amplitude (A) values to self.A
        l = 0.07 # length of module
        Kd = 0.01
        Md = 500
        Bd = 0.01
        A = 0.18
        k = 3
        w_s = self.A * k # spatial frequency, for the curve to helix
        w_t = 2 # temporal frequency, curve of snake backbone (circular)

        J = np.zeros(self.num_modules)
        for i in range(1, self.num_modules+1):
            J[i - 1] = self.A*math.sin(w_s*i*l + w_t*self.t)
        J = J.reshape(-1, 1)
        J = J.transpose()

        effort_reshaped = np.array(self.smoothed_sensor_efforts).reshape(-1, 1)
        effort_0 = np.full((self.num_modules, 1), 0.37).reshape(-1, 1) # calibration efforts, this is basically "0"
        tau_0 = np.matmul(J, effort_reshaped)
        tau = abs(np.matmul(J, effort_reshaped))
        
        # spring mass damper system
        self.vel = (tau - Bd*self.vel[0][0] - Kd*(self.A-A))*(self.dt/Md) + self.vel[0][0]
        self.A = self.vel*self.dt + self.A
        #print(self.A[0][0], np.average(self.smoothed_sensor_efforts))
    
    def helix_climb_ruscelli(self, i):
        k = 3      
        A_lat = 0.18 * (self.t)*0.1 # amplitude
        print(A_lat)
        A_dor = A_lat
        w_s_lat = A_lat * k # spatial frequency, for the curve to helix
        w_s_dor = w_s_lat
        w_t_lat = 2 # temporal frequency, curve of snake backbone (circular)
        w_t_dor = w_t_lat
        l = 0.07 # length of module
        s_i = i * l # distances from head of module for module i

        theta_lat = A_lat*math.sin(w_s_lat*s_i + w_t_lat*self.t)
        theta_dor = -A_dor*math.cos(w_s_dor*s_i + w_t_dor*self.t)

        if (i%4 == 1 or i%4 == 2):
            theta_dor *= -1
            theta_lat *= -1
        

        if (i%2 == 0):
            return theta_dor
        else:
            return theta_lat
    
    def helix_climb_zhen(self, i): # from the "Modelling Rolling Gaits" paper
        self.compliance_p_r()
        m = 0.4 # length of module
        # r = 1.5 # radius
        # p = 1.8 # pitch # looks like we should adjust this for radius change
        # r = 10.0 - self.t*0.5
        # p = 10.0 - self.t*0.5
        (p, r) = (self.p, self.r)
        # s = i * m # arc length
        k = r/(r**2 + p**2) # curvature
        t = p/(r**2 + p**2) # torsion
        # k_cos = k*math.cos(t*s)
        # k_sin = k*math.sin(t*s)
        A = (2*k/t)*math.sin(t*m)
        alpha_cos = 5*A*math.cos(3*self.t + t*m*i) # m = length of module; i = index of joint
        alpha_sin = -5*A*math.sin(3*self.t + t*m*i)
        if (i%4 == 1 or i%4 == 2):
            alpha_cos *= -1
            alpha_sin *= -1
        if (i%2 == 0):
            return alpha_cos
        else:
            return alpha_sin
    
    def compliance_p_r(self): # compliance function, changes p, r
        #print(self.state, round(self.p, 2), round(self.r, 2), round((self.t - self.state_change_time), 2), round(np.average(self.smoothed_sensor_efforts), 2))
        self.get_state()
        if (self.state == "roll_to_pole"):
            self.p = 8.00
            self.r = 8.00
        elif (self.state == "wraparound_pole"):
            self.p = 8.00 - (self.t - self.state_change_time)*0.4
            self.r = 8.00 - (self.t - self.state_change_time)*0.4
        elif (self.state == "climb"):
            return True
        return True
    
    def get_state(self): # get current state of snake for compliance function
        if (self.t < 3.0):
            return True
        elif ((self.state == "roll_to_pole")
                and (self.smoothed_sensor_efforts[7]) > 1.4):
            self.state = "wraparound_pole"
            self.state_change_time = self.t
        # elif ((self.state == "wraparound_pole") and ((self.t - self.state_change_time) > 3)
        #         and abs(self.sensor_efforts[7] > 1.8)):
        #     self.state = "climb"

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
        for i, effort in enumerate(data.effort):
            #effort = abs(effort)
            self.sensor_efforts[i].append(effort)
            self.smoothed_sensor_efforts[i] = running_average_filter(effort, self.sensor_efforts[i], i)
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