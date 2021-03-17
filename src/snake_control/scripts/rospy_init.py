import rospy
if __name__ == '__main__':
    rospy.init_node("main_control")
    try:
        snake_control = SnakeControl(num_modules=16, hz=100, dt=1.0/100)
    except rospy.ROSInterruptException:  pass