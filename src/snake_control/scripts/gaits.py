#!/usr/bin/env python
"""
The node publishes joint position commands to effort position controllers.
The controllers should already be spawned and running and communicating
with the appropriate robot hardware interface.
"""

import rospy
from std_msgs.msg import Float64

import numpy as np

class JointCmds:
    """
    The class provides a dictionary mapping joints to command values.
    """
    def __init__( self, num_mods ) :
        
        self.num_modules = num_mods
        self.jnt_cmd_dict = {}        
        self.joints_list = []
        self.t = 0.0
        
        for i in range(self.num_modules) :
            leg_str='S_'
            if i < 10 :
                leg_str += '0' + str(i)
            else :
                leg_str += str(i)
            self.joints_list += [leg_str]

    def update( self, dt ) :

        self.t += dt
        ## rolling gait ##
        roll_outwards = True
        roll_to_positive = True
        amplitude = 0.2
        TPF = 3 # temporal frequency
        if not roll_to_positive:
            TPF *= -1
        
        if roll_outwards:
            TPF *= -1
        

        for i, jnt in enumerate(self.joints_list) :
            #Wait 5 seconds before moving the snake
            if self.t < 5:
                if i%2 == 0:
                    self.jnt_cmd_dict[jnt] = 0 #amplitude*np.sin(TPO)
                if i%2 == 1:
                    self.jnt_cmd_dict[jnt] = 0
            if self.t > 5:
                if i%2 == 0:
                    self.jnt_cmd_dict[jnt] = amplitude*np.sin( TPF*(self.t - 5))
                if i%2 == 1:
                    self.jnt_cmd_dict[jnt] = amplitude*np.cos( TPF*(self.t - 5) + roll_outwards*np.pi)

            # Account for spiraling
            if i%4 == 1:
                self.jnt_cmd_dict[jnt] = -self.jnt_cmd_dict[jnt]
            if i%4 == 2:
                self.jnt_cmd_dict[jnt] = -self.jnt_cmd_dict[jnt]
        print(self.jnt_cmd_dict)
        
        
        # if round(self.t) == 2:
        #     self.jnt_cmd_dict["S_00"] = np.pi/4
        # elif round(self.t) == 4:
        #     self.jnt_cmd_dict["S_01"] = 0
        # elif round(self.t) == 6:
        #     self.jnt_cmd_dict["S_02"] = -np.pi/4
        # elif round(self.t) == 8:
        #     self.jnt_cmd_dict["S_03"] = 0
        # elif round(self.t) == 10:
        #     self.jnt_cmd_dict["S_04"] = np.pi/4
        # elif round(self.t) == 12:
        #     self.jnt_cmd_dict["S_05"] = 0
        # elif round(self.t) == 14:
        #     self.jnt_cmd_dict["S_06"] = -np.pi/4
        
        # for i, joint in enumerate(self.joints_list):
        #     if i%4 == 0:
        #         self.jnt_cmd_dict[joint] = np.pi/3
        #     elif i%2 == 0:
        #         self.jnt_cmd_dict[joint] = -np.pi/3
        #     else:
        #         self.jnt_cmd_dict[joint] = 0
        
        return self.jnt_cmd_dict

        

def publish_commands( num_modules, hz ):
    pub={}
    ns_str = '/snake'
    cont_str = 'eff_pos_controller'
    for i in range(num_modules) :
        leg_str='S_'
        if i < 10 :
            leg_str += '0' + str(i)
        else :
            leg_str += str(i)
        pub[leg_str] = rospy.Publisher( ns_str + '/' + leg_str + '_'
                                        + cont_str + '/command',
                                        Float64, queue_size=10 )
    rospy.init_node('snake_controller', anonymous=True)
    rate = rospy.Rate(hz)
    jntcmds = JointCmds(num_mods=num_modules)
    while not rospy.is_shutdown():
        jnt_cmd_dict = jntcmds.update(1./hz)
        for jnt in jnt_cmd_dict.keys() :
            pub[jnt].publish( jnt_cmd_dict[jnt] )
        rate.sleep()

"""class DebugPublisher:
    def __init__(self, debug_name = "Debug1"):
        rospy.Publisher(debug_name, String, queue_size=10)"""


if __name__ == "__main__":
    try:
        num_modules = 16        
        hz = 100
        publish_commands( num_modules, hz )
    except rospy.ROSInterruptException:
        pass
