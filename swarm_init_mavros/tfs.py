#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from controller_msgs.msg import PoseStamped
import copy


class OdometryToTFPublisher:
    def __init__(self):
        rospy.init_node("odometry_to_tf_publisher")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.init_offset_x = 0
        self.init_offset_y = 0
        self.init_offset_z = 0

        self.livox_odom = Odometry()

        # Subscribe to odometry messages
        self.livox_odom_sub = rospy.Subscriber("/Odometry", Odometry, self.livox_odom_callback)
        self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.mavros_odom_cb)
        # self.pose_sub = rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.pose_callback) # used by px4
        self.pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped)  # used by px4
        self.initpose_sub = rospy.Subscriber("/initpose", PoseStamped, self.init_callback)

        rospy.loginfo("Odometry to TF publisher initialized")

    def mavros_odom_cb(self, odom_msg):

        transform = TransformStamped()

        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "mavros_local_position_odom"

        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z

        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

    def livox_odom_callback(self, odom_msg):
        self.livox_odom = odom_msg

        # print(self.livox_odom.pose.pose.orientation)

    def pose_callback(self, pose_msg):

        transform = TransformStamped()

        # transform.header.stamp = pose_msg.header.stamp
        # transform.header.frame_id = "world"
        # transform.child_frame_id = "mavros_vision_pose_pose"

        # transform.transform.translation.x = pose_msg.pose.position.x
        # transform.transform.translation.y = pose_msg.pose.position.y
        # transform.transform.translation.z = pose_msg.pose.position.z

        # transform.transform.rotation = pose_msg.pose.orientation

        # self.tf_broadcaster.sendTransform(transform)

    def init_callback(self, pose):
        self.init_offset_x = pose.pose.position.x
        self.init_offset_y = pose.pose.position.y
        self.init_offset_z = pose.pose.position.z
        print(pose)

    def run(self):
        while not rospy.is_shutdown():

            transform = TransformStamped()

            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "world"
            transform.child_frame_id = "mavros_vision_pose_pose"

            transform.transform.translation.x = self.livox_odom.pose.pose.position.x + self.init_offset_x
            transform.transform.translation.y = self.livox_odom.pose.pose.position.y + self.init_offset_y
            transform.transform.translation.z = self.livox_odom.pose.pose.position.z + self.init_offset_z
            transform.transform.rotation.x = self.livox_odom.pose.pose.orientation.x
            transform.transform.rotation.y = self.livox_odom.pose.pose.orientation.y
            transform.transform.rotation.z = self.livox_odom.pose.pose.orientation.z
            transform.transform.rotation.w = self.livox_odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(transform)

            newpose = PoseStamped()
            newpose.header = transform.header
            newpose.pose.position.x = transform.transform.translation.x
            newpose.pose.position.y = transform.transform.translation.y
            newpose.pose.position.z = transform.transform.translation.z
            newpose.pose.orientation = transform.transform.rotation
            self.pose_pub.publish(newpose)

            rospy.sleep(0.1)


def main():
    try:
        publisher = OdometryToTFPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
