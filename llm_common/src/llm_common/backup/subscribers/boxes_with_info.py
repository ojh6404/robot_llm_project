from collections import deque
import threading

import jsk_recognition_msgs.msg
import message_filters
import numpy as np
import rospy
import skrobot

import jsk_robocup_common_msgs.msg
from jsk_robocup_common.models.box import BoundingBox


class BoxesWithInfo(object):

    def __init__(
        self,
        boxes_topic_name,
        class_topic_name,
        classification_topic_name=None,
        approximate_sync=True,
        lifetime=5.0,
    ):
        self.lifetime = lifetime
        self.approximate_sync = approximate_sync
        self.boxes_msg = None
        self.boxes_topic_name = boxes_topic_name
        self.class_topic_name = class_topic_name
        self.classification_topic_name = classification_topic_name
        self.lock = threading.Lock()
        self.history = deque()
        self.subscribe()

    def subscribe(self):
        sub_class = message_filters.Subscriber(
            self.class_topic_name, jsk_recognition_msgs.msg.ClassificationResult
        )
        sub_boxes = message_filters.Subscriber(
            self.boxes_topic_name, jsk_recognition_msgs.msg.BoundingBoxArray
        )
        queue_size = rospy.get_param("~queue_size", 100)
        self.subs = [sub_boxes, sub_class]
        if self.classification_topic_name is not None:
            sub_classification = message_filters.Subscriber(
                self.classification_topic_name,
                jsk_robocup_common_msgs.msg.ClassificationResults,
            )
            self.subs.append(sub_classification)
        if self.approximate_sync:
            slop = rospy.get_param("~slop", 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size, slop
            )
        else:
            sync = message_filters.TimeSynchronizer(self.subs, queue_size)
        sync.registerCallback(self._callback)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _callback(self, boxes_msg, class_msg, classification_msg=None):
        self.boxes_msg = boxes_msg
        self.class_msg = class_msg
        self.classification_msg = classification_msg
        cur_time = rospy.Time.now()
        self.lock.acquire()
        self.history.append((self.boxes_msg, self.class_msg))
        i = 0
        while len(self.history):
            if (cur_time - self.history[i][0].header.stamp).to_sec() > self.lifetime:
                self.history.popleft()
                continue
            break
        self.lock.release()

    def extract_boxes_with_classification(self, target_label_name):
        if self.boxes_msg is None:
            return []
        result_boxes = []
        result_classes = []
        if self.classification_topic_name is None:
            for box, label_name in zip(
                self.boxes_msg.boxes, self.class_msg.label_names
            ):
                if label_name == target_label_name:
                    result_boxes.append(BoundingBox.from_ros_message(box))
        else:
            for box, label_name, result in zip(
                self.boxes_msg.boxes,
                self.class_msg.label_names,
                self.classification_msg.results,
            ):
                if label_name == target_label_name:
                    result_boxes.append(BoundingBox.from_ros_message(box))
                    result_classes.append(result)
        return result_boxes, result_classes

    def wait_target_boxes(self, target_label_name, rate=10):
        rate = rospy.Rate(rate)
        res = []
        while not rospy.is_shutdown() and len(res) == 0:
            res = self.extract_label(target_label_name)
            rate.sleep()
        return res

    def extract_label(self, target_label_name):
        if self.boxes_msg is None:
            return []
        result_boxes = []
        for box, label_name in zip(self.boxes_msg.boxes, self.class_msg.label_names):
            if label_name == target_label_name:
                if (
                    box.pose.orientation.w == 0
                    and box.pose.orientation.x == 0
                    and box.pose.orientation.y == 0
                    and box.pose.orientation.z == 0
                ):
                    box.pose.orientation.w = 1.0
                result_boxes.append(
                    BoundingBox(
                        pos=(
                            box.pose.position.x,
                            box.pose.position.y,
                            box.pose.position.z,
                        ),
                        rot=(
                            box.pose.orientation.w,
                            box.pose.orientation.x,
                            box.pose.orientation.y,
                            box.pose.orientation.z,
                        ),
                        dimensions=(
                            box.dimensions.x,
                            box.dimensions.y,
                            box.dimensions.z,
                        ),
                    )
                )
        return result_boxes

    def extract_boxes(self, target_label_name, voxel_size=0.08, thresh=5):
        positions = []
        self.lock.acquire()
        for bboxes_msg, class_msg in self.history:
            for box, label_name in zip(bboxes_msg.boxes, class_msg.label_names):
                if target_label_name is True or label_name == target_label_name:
                    positions.append(
                        [box.pose.position.x, box.pose.position.y, box.pose.position.z]
                    )
        self.lock.release()
        if len(positions) == 0:
            return np.zeros(shape=(0, 3))
        positions = np.array(positions, dtype=np.float32)

        volume_bounds = np.zeros((3, 2), dtype=np.float64)
        volume_bounds[:, 0] = np.amin(positions, axis=0)
        volume_bounds[:, 1] = np.amax(positions, axis=0)

        volume_dim = (
            np.ceil((volume_bounds[:, 1] - volume_bounds[:, 0]) / voxel_size)
            .copy(order="C")
            .astype(int)
        )
        volume_bounds[:, 1] = volume_bounds[:, 0] + volume_dim * voxel_size
        volume_origin = volume_bounds[:, 0].copy(order="C").astype(np.float32)

        volume_origin = np.array(volume_origin)

        camera_to_voxel_transform = skrobot.coordinates.Coordinates(pos=volume_origin)

        tsdf_vol_cpu = np.zeros(volume_dim).astype(np.float32)

        for pos in positions:
            pos = np.array(pos, dtype=np.float32)
            voxel_pos = (
                camera_to_voxel_transform.inverse_transform_vector(pos) / voxel_size
            )
            voxel_indices = np.array(voxel_pos, dtype=np.int32).T
            try:
                tsdf_vol_cpu[voxel_indices[0], voxel_indices[1], voxel_indices[2]] += 1
            except IndexError:
                pass

        voxels = tsdf_vol_cpu >= thresh
        voxels = np.array(np.where(voxels))
        if voxels.shape[1] == 0:
            return np.zeros(shape=(0, 3))
        camera_object_pos = camera_to_voxel_transform.transform_vector(
            (voxels * voxel_size).T
        )
        return camera_object_pos
