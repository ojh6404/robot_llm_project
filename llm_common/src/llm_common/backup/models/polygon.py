import geometry_msgs.msg
import numpy as np
import shapely.geometry
import skrobot


def area(poly):
    """Calculate area of polygons

    Parameters
    ----------
    poly : numpy.ndarray
        (n, 3) points.
    """
    if len(poly) < 3:
        return 0

    total = [0, 0, 0]
    for i in range(len(poly)):
        vi1 = poly[i]
        if i is len(poly) - 1:
            vi2 = poly[0]
        else:
            vi2 = poly[i + 1]
        prod = np.cross(vi1, vi2)
        total[0] += prod[0]
        total[1] += prod[1]
        total[2] += prod[2]
    result = np.dot(total, unit_normal(poly[0], poly[1], poly[2]))
    return abs(result / 2)


def unit_normal(a, b, c):
    x = np.linalg.det([[1, a[1], a[2]], [1, b[1], b[2]], [1, c[1], c[2]]])
    y = np.linalg.det([[a[0], 1, a[2]], [b[0], 1, b[2]], [c[0], 1, c[2]]])
    z = np.linalg.det([[a[0], a[1], 1], [b[0], b[1], 1], [c[0], c[1], 1]])
    magnitude = (x**2 + y**2 + z**2) ** 0.5
    return (x / magnitude, y / magnitude, z / magnitude)


class Polygon(skrobot.coordinates.CascadedCoords):

    def __init__(self, *args, **kwargs):
        self.points = kwargs.pop("points", [])
        super(Polygon, self).__init__(*args, **kwargs)

    def to_shapely_polygon(self):
        points = self.transform_vector(self._points)
        return shapely.geometry.Polygon((p for p in points))

    @property
    def area(self):
        return area(self._points)

    @property
    def normal(self):
        points = self.worldcoords().transform_vector(self._points[:3])
        n = np.cross(points[1] - points[0], points[2] - points[0])
        n = n / np.linalg.norm(n)
        return n

    @property
    def mean(self):
        return np.mean(self._points, axis=0)

    @property
    def points(self):
        return self._points

    @points.setter
    def points(self, points):
        if len(points) <= 2:
            raise ValueError
        self._points = np.array(points, "f")

    def distance(self, point):
        shapely_polygon = self.to_shapely_polygon()
        distance = shapely_polygon.exterior.distance(shapely.geometry.Point(point))
        return distance

    @staticmethod
    def from_ros_message(msg):
        if isinstance(msg, geometry_msgs.msg.Polygon):
            return Polygon(points=[(p.x, p.y, p.z) for p in msg.points])
        elif isinstance(msg, geometry_msgs.msg.PolygonStamped):
            return Polygon(points=[(p.x, p.y, p.z) for p in msg.polygon.points])
        else:
            raise TypeError("Invalid message {}".format(type(msg)))
