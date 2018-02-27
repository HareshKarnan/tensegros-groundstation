# !/usr/bin/env python
import time
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Float32MultiArray
GPIO.setmode(GPIO.BCM)

# set the appropriate pin connections :
PIN_CLK = 2
PIN_DAT = [3,14]
PIN_CS  = 4

delay = 0.0000005

ns = 2 # number of sensors attached
# totally 10 bits to be extracted from SSI signal
bitcount = 16


def clockup():
    GPIO.output(PIN_CLK,1)
def clockdown():
    GPIO.output(PIN_CLK,0)
def MSB():
    # Most Significant Bit
    clockdown()

def readpos():
    GPIO.output(PIN_CS,0)
    time.sleep(delay*2)
    MSB()
    data = [0]*ns
    
    for i in range(0,bitcount):
        if i<10:
            #print i
            clockup()
            for j in range(0,ns):
                data[j]<<=1  
                data[j]|=GPIO.input(PIN_DAT[j])
            clockdown()
        else:
            for k in range(0,6):
                clockup()
                clockdown()
    GPIO.output(PIN_CS,1)
    return data


def talker():
    pub = rospy.Publisher('encoder_pos', Float32MultiArray, queue_size=1)
    rospy.init_node('encoder', anonymous=True)
    # pin setup done here
    try:
        GPIO.setup(PIN_CLK, GPIO.OUT)
        GPIO.setup(PIN_DAT[:], GPIO.IN)
        GPIO.setup(PIN_CS, GPIO.OUT)
        GPIO.output(PIN_CS, 1)
        GPIO.output(PIN_CLK, 1)
        print "GPIO configuration enabled"
    except:
        print "ERROR. Unable to setup the configuration requested"
        # wait some time to start
    time.sleep(0.5)
    while not rospy.is_shutdown():
        rospy.loginfo(readpos())
        pub.publish(readpos())


if __name__ == '__main__' :
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Cleaning GPIO ports...")
        GPIO.cleanup()
        rospy.loginfo("GPIO ports cleaned...")
