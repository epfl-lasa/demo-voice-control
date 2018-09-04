#! /usr/bin/env python
import rospy
from ros_boilerplate.boilerplate_class import BoilerPlate


def run():
    rospy.init_node('boilerplate')
    boilerplate = BoilerPlate()

    # NOTE: It's a good idea to sleep a little bit after creating all
    # publishers and subscribers. This gives the node time to register with
    # master and prevents dropping the first message(s).
    rospy.sleep(0.5)
    rospy.loginfo('Node started')

    rospy.loginfo('Running until shutdown (Ctrl-C).')
    while not rospy.is_shutdown():
        boilerplate.run_once()
        rospy.sleep(0.5)

    rospy.loginfo('Node finished')

if __name__ == '__main__':
    # TODO add argument parsing.
    run()
