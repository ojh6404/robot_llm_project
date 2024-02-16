from collections import defaultdict
from enum import IntEnum

import geometry_msgs.msg
import jsk_recognition_msgs.msg
import message_filters
import numpy as np
import rospy
import shapely.geometry
import skrobot
from skrobot.coordinates import Coordinates
from skrobot.coordinates import rotate_points

from jsk_robocup_common.models.polygon import Polygon
from jsk_robocup_common.models.box import BoundingBox


def random_points_within(poly, num_points, box_polygons=None):
    box_polygons = box_polygons or []
    try:
        min_x, min_y, max_x, max_y = poly.bounds
    except ValueError:
        return []

    points = []

    while len(points) < num_points:
        random_point = shapely.geometry.Point(
            [np.random.uniform(min_x, max_x),
             np.random.uniform(min_y, max_y)])
        if (random_point.within(poly)):
            valid = True
            for box_polygon in box_polygons:
                if random_point.within(box_polygon):
                    valid = False
                    break
            if valid:
                points.append(random_point)

    return points


def points_within(poly, box_polygons=None, margin=0.05):
    box_polygons = box_polygons or []
    try:
        min_x, min_y, max_x, max_y = poly.bounds
    except ValueError:
        return []
    points = []

    for y in np.arange(min_y, max_y, margin):
        for x in np.arange(min_x, max_x, margin):
            point = shapely.geometry.Point([x, y])
            if not point.within(poly):
                continue
            valid = True
            for box_polygon in box_polygons:
                if point.within(box_polygon):
                    valid = False
                    break
            if valid:
                points.append(point)

    return points


class PlacementFinder(object):

    def __init__(self,
                 polygon_topic_name,
                 coefficients_topic_name,
                 boxes_topic_name,
                 approximate_sync=False,
                 queue_size=10,
                 slop=0.1):
        super(PlacementFinder, self).__init__()
        self.queue_size = queue_size
        self.approximate_sync = approximate_sync
        self.slop = slop

        self.polygon_topic_name = polygon_topic_name
        self.coefficients_topic_name = coefficients_topic_name
        self.boxes_topic_name = boxes_topic_name

        self.polygons_msg = None
        self.coeffs_msg =None
        self.boxes_msg = None
        self.subscribe()

    def subscribe(self):
        queue_size = self.queue_size
        sub_polygon = message_filters.Subscriber(
            self.polygon_topic_name,
            jsk_recognition_msgs.msg.PolygonArray,
            queue_size=1)
        sub_coefficients = message_filters.Subscriber(
            self.coefficients_topic_name,
            jsk_recognition_msgs.msg.ModelCoefficientsArray,
            queue_size=1)

        sub_boxes = message_filters.Subscriber(
            self.boxes_topic_name,
            jsk_recognition_msgs.msg.BoundingBoxArray,
            queue_size=1)
        self.subs = [sub_polygon, sub_coefficients, sub_boxes]
        if self.approximate_sync:
            slop = self.slop
            sync = message_filters.ApproximateTimeSynchronizer(
                fs=self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                fs=self.subs, queue_size=queue_size)
        sync.registerCallback(self._cb_with_boxes)

    def unsubscribe(self):
        for s in self.subs:
            s.unregister()

    def _cb_with_boxes(self, polygons_msg, coeffs_msg, boxes_msg):
        self.polygons_msg = polygons_msg
        self.coeffs_msg = coeffs_msg
        self.boxes_msg = boxes_msg

        if polygons_msg.header.frame_id != boxes_msg.header.frame_id:
            raise ValueError

    def get_polygons(self, with_message=False):
        polygons_msg = self.polygons_msg
        coeffs_msg = self.coeffs_msg
        boxes_msg = self.boxes_msg
        output_polygons = []
        for index, (coeff, polygon) in enumerate(zip(
                coeffs_msg.coefficients,
                polygons_msg.polygons)):
            output_polygons.append(Polygon.from_ros_message(polygon))
        if with_message is False:
            return output_polygons
        else:
            return output_polygons, polygons_msg, coeffs_msg, boxes_msg

    def get_rects(self,
                  polygons_msg=None,
                  coeffs_msg=None,
                  boxes_msg=None,
                  indices=None):
        polygons_msg = polygons_msg or self.polygons_msg
        coeffs_msg = coeffs_msg or self.coeffs_msg
        boxes_msg = boxes_msg or self.boxes_msg

        rect_polygons = []
        for index, (coeff, polygon) in enumerate(
                zip(coeffs_msg.coefficients,
                    polygons_msg.polygons)):
            if indices is not None and index not in indices:
                continue

            a, b, c, d = coeff.values
            points = np.array(
                [[point.x, point.y, point.z]
                 for point in polygon.polygon.points],
                dtype=np.float32)
            normal = [a, b, c]
            projected_points = rotate_points(
                points,
                normal,
                [0, 0, 1])
            shapely_polygon = shapely.geometry.Polygon(
                projected_points)
            rect = shapely_polygon.minimum_rotated_rectangle
            x, y = rect.exterior.xy
            rect_polygon_2d = - np.ones((5, 3), 'f') * d
            rect_polygon_2d[:, 0] = x
            rect_polygon_2d[:, 1] = y
            min_x = np.min(np.abs(rect_polygon_2d[1:, 0] - rect_polygon_2d[:-1, 0]))
            min_y = np.min(np.abs(rect_polygon_2d[1:, 1] - rect_polygon_2d[:-1, 1]))
            rect_polygon = rotate_points(
                rect_polygon_2d,
                [0, 0, 1],
                normal)
            rect_polygons.append(rect_polygon)
        return rect_polygons

    def get_poses(self,
                  polygons_msg=None,
                  coeffs_msg=None,
                  boxes_msg=None,
                  margin=0.02, indices=None):
        polygons_msg = polygons_msg or self.polygons_msg
        coeffs_msg = coeffs_msg or self.coeffs_msg
        boxes_msg = boxes_msg or self.boxes_msg

        plane_id_to_boxes = defaultdict(list)
        for box in boxes_msg.boxes:
            plane_id_to_boxes[box.label].append(box)

        output_poses_array = []
        output_polygons = []
        for index, (coeff, polygon) in enumerate(zip(
                coeffs_msg.coefficients,
                polygons_msg.polygons)):
            if indices is not None and index not in indices:
                continue
            output_polygons.append(Polygon.from_ros_message(polygon))

            a, b, c, d = coeff.values
            points = np.array(
                [[point.x, point.y, point.z]
                 for point in polygon.polygon.points],
                dtype=np.float32)

            normal = np.array([a, b, c])
            projected_points = rotate_points(
                points,
                normal,
                [0, 0, 1])
            shapely_polygon = shapely.geometry.Polygon(
                projected_points)
            boxes = plane_id_to_boxes[index]

            shapely_box_polygons = []
            for box in boxes:
                box = BoundingBox.from_ros_message(box)
                projected_vertices = rotate_points(
                    box.vertices,
                    normal,
                    [0, 0, 1])
                box_polygon = shapely.geometry.MultiPoint(
                    projected_vertices).convex_hull

                shapely_box_polygons.append(box_polygon.buffer(margin))

            points = points_within(shapely_polygon.buffer(-margin),
                                   box_polygons=shapely_box_polygons)

            orientation_coords = skrobot.coordinates.Coordinates()
            skrobot.coordinates.geo.orient_coords_to_axis(
                orientation_coords, - normal)
            q_wxyz = orientation_coords.quaternion
            output_poses = []
            for p in points:
                x, y, z = rotate_points(
                    np.array([p.x, p.y, -d]),
                    [0, 0, 1],
                    normal)[0]
                output_poses.append(Coordinates(
                    pos=(x, y, z),
                    rot=q_wxyz))
            output_poses_array.append(output_poses)
        return output_poses_array, output_polygons
