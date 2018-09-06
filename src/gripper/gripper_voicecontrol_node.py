#!/usr/bin/env python

"""This node is a voice control for easy kinesthetic teachin purposed in ROS 
in open/close of robotiq 2-finger gripper and start/halt data recording
""" 
import rospy
import argparse
from gripper_voicecontrol_class import ASRControl


def run_node(): 
    # Initialize Node
    rospy.init_node('teach_voice_control_node')

    # Parse ros params
    model   = rospy.get_param('~model')
    lexicon = rospy.get_param('~lexicon')
    kwlist  = rospy.get_param('~kwlist')
    rospub  = rospy.get_param('~rospub')

    # Run VoiceControl Class
    voicecontrol = ASRControl(model, lexicon, kwlist, rospub)

    rospy.loginfo('Running until shutdown (Ctrl-C).')
    while not rospy.is_shutdown():
       voicecontrol.shutdown()
       rospy.sleep(0.5)

    rospy.loginfo('Node finished')

                      

if __name__ == '__main__':
    run_node()

                   