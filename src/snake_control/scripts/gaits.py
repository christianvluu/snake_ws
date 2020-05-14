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

        ## sidewinding gait ##
        # spatial frequency
        spat_freq = np.pi/4
        
        # temporal phase offset between horizontal and vertical waves
        TPO = np.pi/4

        # amplitude
        A_even = 1
        A_odd = 0.7

        # direction
        d = 3

        ## rolling gait ##
        '''
        spat_freq = 0
        TPO = np.pi/2
        A_even = 0.25
        A_odd - 0.25
        d = 0.5
        '''

        for i, jnt in enumerate(self.joints_list) :
            # Wait 10 seconds before moving the snake
            if self.t < 10:
                if i%2 == 0:
                    self.jnt_cmd_dict[jnt] = A_even*np.sin( (i%2)*TPO + i*spat_freq )
                if i%2 == 1:
                    self.jnt_cmd_dict[jnt] = A_odd*np.sin( (i%2)*TPO + i*spat_freq )
            else:
                if i%2 == 0:
                    self.jnt_cmd_dict[jnt] = A_even*np.sin( d*(self.t - 10) + (i%2)*TPO + i*spat_freq )
                if i%2 == 1:
                    self.jnt_cmd_dict[jnt] = A_odd*np.sin( d*(self.t - 10) + (i%2)*TPO + i*spat_freq )

            # Account for spiraling
            if i%4 > 1:
                self.jnt_cmd_dict[jnt] = -self.jnt_cmd_dict[jnt]
                
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


if __name__ == "__main__":
    try:
        num_modules = 16        
        hz = 100
        publish_commands( num_modules, hz )
    except rospy.ROSInterruptException:
        pass
