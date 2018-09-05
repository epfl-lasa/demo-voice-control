#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_s_model_control')
import rospy
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from time import sleep


def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg.SModel_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if char == 'r':
        command = outputMsg.SModel_robot_output();
        command.rACT = 0

    if char == 'c':
        command.rPRA = 255

    if char == 'o':
        command.rPRA = 0

    if char == 'b':
        command.rMOD = 0
        
    if char == 'p':
        command.rMOD = 1
        
    if char == 'w':
        command.rMOD = 2
        
    if char == 's':
        command.rMOD = 3

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPRA = int(char)
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255
            
    if char == 'l':
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0

            
    if char == 'i':
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255
            
    if char == 'd':
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0

    return command
        

def askForCommand(command):
    """Ask the user for a command to send to the gripper."""    

    currentCommand  = 'Simple S-Model Controller\n-----\nCurrent command:'
    currentCommand += ' rACT = '  + str(command.rACT)
    currentCommand += ', rMOD = ' + str(command.rMOD)
    currentCommand += ', rGTO = ' + str(command.rGTO)
    currentCommand += ', rATR = ' + str(command.rATR)
    currentCommand += ', rPRA = ' + str(command.rPRA)
    currentCommand += ', rSPA = ' + str(command.rSPA)
    currentCommand += ', rFRA = ' + str(command.rFRA)
    print currentCommand

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 'b: Basic mode\n'
    strAskForCommand += 'p: Pinch mode\n'
    strAskForCommand += 'w: Wide mode\n'
    strAskForCommand += 's: Scissor mode\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'
    
    strAskForCommand += '-->'

    return raw_input(strAskForCommand)

def publisher():
    """Main loop which requests new commands and publish them on the SModelRobotOutput topic."""

    rospy.init_node('SModelSimpleController')
    
    pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output)

    command = outputMsg.SModel_robot_output();

    while not rospy.is_shutdown():

        command = genCommand(askForCommand(command), command)            
        
        pub.publish(command)

        rospy.sleep(0.1)
                        

if __name__ == '__main__':
    publisher()
