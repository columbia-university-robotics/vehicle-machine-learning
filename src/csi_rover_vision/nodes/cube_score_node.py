#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import PointStamped
from srcp2_msgs.srv import AprioriLocationSrv
from geometry_msgs.msg import PoseStamped


class CubeScoreNode:

    def __init__(self):

        rospy.init_node('cube_score_node', anonymous=True)
        rospy.Subscriber("/cube_position", PointStamped, callback=self.cube_detection_callback)
        self.odom_frame = "odom"
        self.camera_frame = "/scout_1_tf/left_camera_optical"
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.odom_frame, self.camera_frame, rospy.Time(), rospy.Duration(3))
        self.cube_position = None

        rospy.spin()

    def cube_detection_callback(self, data):
        self.cube_position = data
        self.report_cube_position()

    def report_cube_position(self):

        if self.cube_position is not None:

            self.listener.waitForTransform(self.odom_frame, self.camera_frame, rospy.Time.now(), rospy.Duration(3))
            position = self.listener.transformPoint(self.odom_frame, self.cube_position)

            rospy.wait_for_service("/apriori_location_service")
            rospy.loginfo("[REPORT]: Waiting to report cube position")
            try:
                report_cube_position = rospy.ServiceProxy("/apriori_location_service", AprioriLocationSrv)
                response = report_cube_position(position.point)
                print(response)
                if response is True:
                    print("points earned")
                    rospy.signal_shutdown("Job finished")
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)


if __name__ == '__main__':
    try:
        CubeScoreNode()
    except rospy.ROSInterruptException:
        pass
