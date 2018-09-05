import rospy
from std_msgs.msg import String

# Libraries for voice commands
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio

# Libraries for gripper commands
import roslib; roslib.load_manifest('robotiq_s_model_control')
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from time import sleep


class ASRControl(object):
    """Simple voice control interface for ROS command

    Attributes:
        model: model path
        lexicon: pronunciation dictionary
        kwlist: keyword list file
        pub: where to send commands (default: 'mobile_base/commands/velocity')

    """
    def __init__(self, model, lexicon, kwlist, pub_string):
        # initialize ROS        
        self.msg_string       = String()            
        self.msg_gripper      = outputMsg.SModel_robot_output();
        rospy.on_shutdown(self.shutdown)

        # you may need to change publisher destination depending on what you run
        self.pub_string  = rospy.Publisher(pub_string, String, queue_size=10)
        self.pub_gripper = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output)
        rospy.sleep(2) 

        # Activate gripper     
        self.msg_gripper = self.genCommand("r", self.msg_gripper)     
        self.pub_gripper.publish(self.msg_gripper)
        rospy.sleep(1) 

        print "Activating Gripper\n"
        self.msg_gripper = self.genCommand("a", self.msg_gripper) 
        self.pub_gripper.publish(self.msg_gripper)
        rospy.sleep(1) 


        # initialize pocketsphinx
        config = Decoder.default_config()
        config.set_string('-hmm', model)
        config.set_string('-dict', lexicon)
        config.set_string('-kws', kwlist)   

        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()

        self.decoder = Decoder(config)
        self.decoder.start_utt()

        while not rospy.is_shutdown():
            buf = stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                break
            self.parse_asr_result()

    def parse_asr_result(self):
        """
        publish commands to message based on ASR hypothesis
        """
        if self.decoder.hyp() != None:
            print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                for seg in self.decoder.seg()])
            print ("Detected keyphrase, restarting search")
            seg.word = seg.word.lower()
            self.decoder.end_utt()
            self.decoder.start_utt()
            # Sending recognized word as string
            if seg.word.find("start") > -1:
                self.msg_string.data = 'start'                
            if seg.word.find("halt") > -1:
                self.msg_string.data = 'halt'
            
            if seg.word.find("open") > -1:                
                self.msg_string.data = 'open'            
                self.msg_gripper = self.genCommand("o", self.msg_gripper)     
                self.pub_gripper.publish(self.msg_gripper)
            if seg.word.find("close") > -1:
                self.msg_string.data = 'close'
                self.msg_gripper = self.genCommand("c", self.msg_gripper)     
                self.pub_gripper.publish(self.msg_gripper)

            if seg.word.find("robot") > -1:
                self.msg_string.data = 'robot'

            self.pub_string.publish(self.msg_string)

    def genCommand(self, char, command):
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

        return command


    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop ASRControl")
        self.pub_string.publish(String())
        rospy.sleep(1)

# No main/run code, see boilerplate_node.py for the node execution.
