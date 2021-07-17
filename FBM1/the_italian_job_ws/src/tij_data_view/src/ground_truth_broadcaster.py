#!/usr/bin/env python
import rospy

import tf
from geometry_msgs.msg import PoseStamped


class GroundTruthTransformPublisher(object):
    def __init__(self):
        # transform broadcaster
        self._tf_broadcaster = tf.TransformBroadcaster()

        # subscriber
        self._pose_sub = rospy.Subscriber('/vicon_client/METRICS/pose',
                                          PoseStamped,
                                          self._pose_callback)

    def _pose_callback(self, msg):
        self._tf_broadcaster.sendTransform(
            (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w),
            rospy.Time.now(),
            "vicon_sensor",
            "vicon")

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('node_name')
    GroundTruthTransformPublisher().run()


main()
