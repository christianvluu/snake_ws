import rospy
from sensor_msgs.msg import JointState
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime
import numpy as np

a = np.array([1,2,3,4,5,6,7,8,9,10])
a_reshaped = a.reshape(-1, 1)
print(a_reshaped)
a_reshaped_transposed = a_reshaped.transpose()
print(a_reshaped_transposed)
print(a.shape[0])


# efforts_0 = []
# x_data = []

# def draw(x_data, y_data):
#     x_data.append(len(x_data)+1)
#     if (len(x_data) == 500):
#         plt.plot(x_data, y_data)
#         plt.show()
#     else:
#         print("Drawing..." + str(500-len(x_data)))

# def callback(data):
#     effort_list = list(data.effort)
#     efforts_0.append(effort_list[6])
#     #draw(x_data, efforts_0)
    
#     for i, effort in enumerate(effort_list):
#         print str(i) + ": " + str(round(data.effort[i], 4)) # extract efforts of joints
    
    
# def listener():
#     rospy.init_node('join_state_listener', anonymous=True)

#     rospy.Subscriber("/snake/joint_states", JointState, callback)

#     rospy.spin()

# if __name__ == '__main__':
#     listener()