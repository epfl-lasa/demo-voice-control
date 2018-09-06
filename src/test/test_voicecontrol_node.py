#!/usr/bin/env python

"""This node is a simple demonstration of voice control for ROS 
"""

import rospy
import argparse
from test_voicecontrol_class import ASRControl


def run(args):
      
    voicecontrol = ASRControl(args.model, args.lexicon, args.kwlist, args.rospub)

    rospy.loginfo('Running until shutdown (Ctrl-C).')
    while not rospy.is_shutdown():
        voicecontrol.shutdown()
        rospy.sleep(0.5)

    rospy.loginfo('Node finished')

if __name__ == '__main__':
    
    # TODO add argument parsing from yaml or launch file!
    parser = argparse.ArgumentParser(
        description='Voice control test using pocketsphinx.')
    
    parser.add_argument('--model', type=str,
        default='/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k',
        help='''acoustic model path
        (default: /usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k)''')
    
    # You must modify these paths
    parser.add_argument('--lexicon', type=str,
        default='/home/nbfigueroa/proj/catkin_ws_lags/src/demo-voice-control/commands/voice_cmd_test.dic',
        help='''pronunciation dictionary
        (default: voice_cmd.dic)''')
    
    # You must modify these paths
    parser.add_argument('--kwlist', type=str,
        default='/home/nbfigueroa/proj/catkin_ws_lags/src/demo-voice-control/commands/voice_cmd_test.kwlist',
        help='''keyword list with thresholds
        (default: voice_cmd.kwlist)''')
    
    parser.add_argument('--rospub', type=str,
        default='demo_voice_control/command',
        help='''ROS publisher destination
        (default: demo_voice_control/command)''')
    
    args = parser.parse_args()
    run(args)
