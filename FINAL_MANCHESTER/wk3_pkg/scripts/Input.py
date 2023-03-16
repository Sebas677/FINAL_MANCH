#!/usr/bin/env python
import rospy
import numpy as np
from wk3_pkg.msg import signal_msg
from wk3_pkg.msg import set_point_msg
from std_msgs.msg import Float32

flagSq= rospy.get_param("/flagSq", 0)
flagSt= rospy.get_param("/flagSt", 0)

if __name__=='__main__':
       ## Declare the new message to be used   
    setpoint_pub=rospy.Publisher("/setpoint", Float32,queue_size=10)
    rospy.init_node("signal_generator")
    rate = rospy.Rate(50)
    init_time = rospy.get_time()
    msg = Float32()


    while not rospy.is_shutdown():

        time = rospy.get_time()-init_time
        setPoint= rospy.get_param("/setP", 1)
        rotation= rospy.get_param("/rotate", "left")
        type_sig= rospy.get_param("/type", "step")
    

        rospy.loginfo("Rotation to %s of  %s wave type",rotation, type_sig)

        if type_sig=='sine':
            signal = setPoint*np.sin(time)

        elif type_sig=='step':
            signal=0
            if flagSt>5:
                signal=setPoint
            else:
                signal=0
                flagSt= flagSt+1


        else:
            rospy.loginfo("Not an option")


        setpoint_pub.publish(signal)

        rospy.loginfo("The signal value is: %f at a time %f", signal, time)
        rate.sleep()