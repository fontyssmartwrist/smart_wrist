#!/usr/bin/env python
import rospy, time
import math
from std_msgs.msg import String, Float64, Int32, UInt32, Time
from sensor_msgs.msg import JointState


def talker():
    rospy.init_node('joint_pose', anonymous=False)
    # subscibtions and publishing    
    pub=rospy.Publisher('/joint_states', JointState,queue_size=10)

    rate=rospy.Rate(1000) 
    pi=3.14159265359
    size=pi/100000
    step1=0
    

    while step1<pi: 
        msg=JointState()
        step2=0
        while step2<pi:
            step3=0
            while step3<pi:
                step4=0
                while step4<pi:
                    msg.header.stamp=rospy.get_rostime()
                    msg.name=["joint0", "joint1", "joint2", "joint3"]
                    msg.position= [step1, step1, step1, step1]
                    msg.velocity= [0, 0, 0, 0]

                    pub.publish(msg)
                    step4+=size
                    rate.sleep()
                step3+=size
                rate.sleep()
            step2+=size
            rate.sleep()
        step1+=size
        rate.sleep()                    

    
if __name__ == '__main__':
    try:
       talker()
    except rospy.ROSInterruptException: 
        pass