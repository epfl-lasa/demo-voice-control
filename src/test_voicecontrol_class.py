import rospy
from std_msgs.msg import String
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio


class ASRControl(object):
    """Simple voice control interface for ROS command

    Attributes:
        model: model path
        lexicon: pronunciation dictionary
        kwlist: keyword list file
        pub: where to send commands (default: 'mobile_base/commands/velocity')

    """
    def __init__(self, model, lexicon, kwlist, pub):
        # initialize ROS
        self.msg = String

        rospy.init_node('test_voice_cmd')
        rospy.on_shutdown(self.shutdown)

        # you may need to change publisher destination depending on what you run
        self.pub_ = rospy.Publisher(pub, String, queue_size=10)

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
            # you may want to modify the main logic here
            if seg.word.find("back") > -1:
                self.pub_.publish('back')

            if seg.word.find("forward") > -1:
                self.pub_.publish('forward')

            if seg.word.find("left") > -1:
                self.pub_.publish('left')

            if seg.word.find("right") > -1:
                self.pub_.publish('right')

            if seg.word.find("move") > -1:
                self.pub_.publish('move')
            
            if seg.word.find("stop") > -1:
                self.pub_.publish('stop')

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish(String())
        rospy.sleep(1)

# No main/run code, see boilerplate_node.py for the node execution.
