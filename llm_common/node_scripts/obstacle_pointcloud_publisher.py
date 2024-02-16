#!/usr/bin/env python3

## pointcloud を 引き継いで publish し続ける

import numpy as np
from datetime import datetime
import rospy
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.point_cloud2 import read_points, create_cloud
from skrobot.coordinates import Coordinates


def do_transform_cloud(cloud, transform):
    base_to_map = tf_pose_to_coords(transform)
    points_out = []
    points_in_map = np.array(list(read_points(cloud)))[::2]
    points_in_base = base_to_map.inverse_transformation().inverse_transform_vector(points_in_map[:, :3])
    points_out = np.concatenate([points_in_base, points_in_map[:, 3].reshape(-1, 1)], axis=1)
    res = create_cloud(transform.header, cloud.fields, points_out)
    return res


class ObstacleCloudPublisher(object):
    def __init__(self):
        self.start = False
        self.exec_flag = False
        self.pc2_msg_transformed = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cloud_pub = rospy.Publisher("obstacle_cloud", PointCloud2, queue_size=50)
        self.cloud_sub = rospy.Subscriber("/head_docker/detic_cluster_point_indices_decomposer/debug_output",
                                           PointCloud2,
                                           self.cloud_cb,
                                           queue_size=1)
        self.start_service = rospy.Service('obstacle_start_request', Trigger, self.start_service_cb)
        self.end_service = rospy.Service('obstacle_end_request', Trigger, self.end_service_cb)

        self.refrence_frame = "map"
        self.publish_frame = "base_range_sensor_link"

        rospy.loginfo("PointCloud Take Over Server initialized")

        duration = rospy.Duration(0.05)
        # duration = rospy.Duration(0.1)
        self.timer = rospy.Timer(duration, self.timer_cb)

    def cloud_cb(self, msg: PointCloud2):
        if self.start and not self.exec_flag:
            # Lookup the transform from the map to the PointCloud2 message
            try:
                self.transform = self.tf_buffer.lookup_transform(self.refrence_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr('Failed to lookup transform: {}'.format(e))
                return
            # Transform the PointCloud2 message to the map frame
            self.pc2_msg_transformed = do_transform_cloud(msg, self.transform)
            self.pc2_msg_transformed.header.stamp = rospy.Time.now()
            self.pc2_msg_transformed.header.frame_id = self.publish_frame
            self.cloud_pub.publish(self.pc2_msg_transformed)

            self.exec_flag = True


    def start_service_cb(self, req):
        rospy.loginfo('Started publishing PointCloud2 message.')
        self.start = True
        return TriggerResponse(success=True)

    def end_service_cb(self, req):
        rospy.loginfo('Stopped publishing PointCloud2 message.')
        self.start = False
        self.exec_flag = False
        return TriggerResponse(success=True)

    def timer_cb(self, event):
        if self.start and self.exec_flag:
            start = datetime.now()
            # Lookup the transform from the map to the PointCloud2 message
            stamp = rospy.Time.now()
            try:
                self.transform = self.tf_buffer.lookup_transform(self.publish_frame, self.refrence_frame,
                                                                 rospy.Time(0),
                                                                 # stamp,
                                                                 # rospy.Time.now(),
                                                                 rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr('Failed to lookup transform: {}'.format(e))
                return
            # end = datetime.now()
            # print("lookup")
            # print(end - start)
            self.pc2_msg_transformed_pub = do_transform_cloud(self.pc2_msg_transformed, self.transform)
            # end = datetime.now()
            # print("do_transform_cloud")
            # print(end - start)
            self.pc2_msg_transformed_pub.header.stamp = stamp
            self.cloud_pub.publish(self.pc2_msg_transformed_pub)
            # end = datetime.now()
            # print("publish")
            # print(end - start)

if __name__ == "__main__":
    rospy.init_node('pointcloud_take_over_publisher')
    server = ObstacleCloudPublisher()
    rospy.spin()
