#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    # topic name : motors_command
    pub = rospy.Publisher('motors_command', Float32MultiArray, queue_size=6)
    # name of the node : motor_comms_test
    rospy.init_node('motor_comms_test',anonymous = False)
    while not rospy.is_shutdown():
        # get input from user for the motors and publish it
        a = map(float,raw_input().split())
        if len(a)==6:
            pass
        else:
            a = [0,0,0,0,0,0]

        rospy.loginfo('Publishing motor test data : ' + str(a))
        pub.publish(Float32MultiArray(data=a))


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Closing motor_comms_test")
