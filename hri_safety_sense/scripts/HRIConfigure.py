#! /usr/bin/python

import sys
import os
import rospy

from hri_safety_sense.msg import KeyValueResp
from hri_safety_sense.msg import RemoteStatus
from hri_safety_sense.srv import KeyValue
from hri_safety_sense.srv import MessageConfigure

def keyValueCallback(msg):
    print(msg)
    pass

def remoteStateCallback(msg):
    print(msg)
    pass


if __name__ == "__main__":
    rospy.init_node("HRIConfigure", anonymous=True)
    
    rospy.Subscriber("/safety/key_values", KeyValueResp, keyValueCallback)
    rospy.Subscriber("/safety/remote_status", RemoteStatus, remoteStateCallback)

    configure_message = rospy.ServiceProxy("/safety/config_message",MessageConfigure)
    request_key_value = rospy.ServiceProxy("/safety/request_key",KeyValue)
    
    rospy.sleep(10)
    request_key_value(1,0);
    rospy.sleep(1)
    request_key_value(2,0);
    rospy.sleep(1)
    request_key_value(3,0);
    rospy.sleep(1)
    configure_message(0x22,1,1000);
    rospy.spin()