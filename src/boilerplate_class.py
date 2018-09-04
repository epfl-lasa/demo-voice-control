import rospy
from std_msgs.msg import String


class BoilerPlate(object):
    """
    This class just talks to itself.
    """

    def __init__(self):
        self._pub = rospy.Publisher('foo_py', String, queue_size=10)
        self._sub = rospy.Subscriber('foo_py', String, self._string_cb)
        self._count = 0

    def _string_cb(self, msg):
        """
        Callback: just prints what it heard.
        """
        rospy.loginfo('Received {}'.format(msg.data))

    def run_once(self):
        """
        Main doer method. Blocking here is generally bad.
        """
        self._count += 1
        self._pub.publish('Hello world #{}'.format(self._count))


# No main/run code, see boilerplate_node.py for the node execution.
