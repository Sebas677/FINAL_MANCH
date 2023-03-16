#!/usr/bin/env python
import rospy
import numpy as np
from wk3_pkg.msg import output_msg
from wk3_pkg.msg import signal_msg
from wk3_pkg.msg import set_point_msg
from wk3_pkg.msg import rpm_msg
from std_msgs.msg import Float32

#Setup parameters, variables and callback functions here (if required)
Kp = rospy.get_param("/kp",1.0) #reference 0.0875
Ki=rospy.get_param("/ki",1.0) #reference 0.087
Kd=rospy.get_param("/kd", 1.0 ) #reference 0.000000001
integral=rospy.get_param("/integral",0.0)
last_error=rospy.get_param("/last_error",0.0)
last_time=rospy.get_param("/last_time",0.0)
set_point=rospy.get_param("/set_point",0.0)
Angular=rospy.get_param("/Angular",0.0)


def set_point_callback(msg):
  global set_point
  set_point=msg.data


def angular_callback(msg):
  global Angular
  Angular=msg.data
  

#Stop Condition
def stop():
#Setup the stop message (can be the same as the control message)
 print("Stopping")


if __name__=='__main__':
   #Initialise and Setup node
   rospy.init_node("controller")
   rate = rospy.Rate(150)
   rospy.on_shutdown(stop)

   #Setup Publishers and subscribers here
   rospy.Subscriber("/setpoint",Float32,set_point_callback)
   rospy.Subscriber("/rpm",Float32,angular_callback)
   output_pub=rospy.Publisher("/u_out",  Float32,queue_size=1)
   msg1=Float32()

   #Run the node
   while not rospy.is_shutdown():


     #Write your code here
     #calculate the error
     rospy.loginfo("ki: %f",Ki)
     rospy.loginfo("kp: %f",Kp)
     error=set_point-Angular
     rospy.loginfo("error: %f",error)
     current_time=rospy.get_time()
     dt= current_time-last_time

     integral+=error*dt
     derivative=(error-last_error)/dt
     output=Kp*error+Ki*integral
     #output=Kp*error
     #output=Kp*error+Ki*integral+Kd*derivative
     output_pub.publish(output)


     rospy.loginfo("u(t): %f",output)
    
     last_time=current_time
     last_error=error

     rate.sleep()