#!/usr/bin/env python

from geometry_msgs.msg import PointStamped as Point
from rospy import Publisher as Pub
from rospy import sleep, spin, init_node


class RoutePlanner:
    def __init__(self):
        self.pub = Pub('/clicked_point', Point, queue_size=100)

    def _publish_point(self, x, y):
        sleep(0.5)
        p = Point()
        p.header.frame_id = 'map'
        p.point.x = float(x)
        p.point.y = float(y)
        self.pub.publish(p)

    def go(self, x, y):
        self._publish_point(x - 1, y - 1)
        self._publish_point(x + 1, y - 1)
        self._publish_point(x, y + 1)
        self._publish_point(x - 1, y - 1)

        self._publish_point(x, y)

    def explore(self):
        self._publish_point(-100, -100)
        self._publish_point(-100, 100)
        self._publish_point(100, 100)
        self._publish_point(100, -100)
        self._publish_point(-100, -100)
        self._publish_point(0, 0)


if __name__ == '__main__':
    init_node(RoutePlanner.__name__)

    target = RoutePlanner()
    target.go(16, -21)

    spin()
