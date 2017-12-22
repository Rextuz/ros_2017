#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from rospy import Duration


class TargetSelector:
    def __init__(self):
        self.pub = rospy.Publisher('/clicked_point', PointStamped,
                                   queue_size=100)

    def _publish_point(self, x, y):
        rospy.sleep(0.5)
        p = PointStamped()
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


class LocationFinder:
    def __init__(self):
        self.position = None
        rospy.Subscriber('/odom', Odometry, self._callback)

    def _callback(self, odometry):
        self.position = odometry.pose.pose.position


class Trace:
    def __init__(self):
        self.pub = rospy.Publisher('/trace', Path, queue_size=100)
        self.loc = LocationFinder()
        self.path = Path()
        self.path.header.frame_id = 'odom'
        rospy.Timer(Duration.from_sec(1), self._path)

    def _path(self, _):
        p = PoseStamped()
        p.header.frame_id = 'odom'
        p.pose.position.x = self.loc.position.x
        p.pose.position.y = self.loc.position.y
        p.pose.position.z = self.loc.position.z
        self.path.poses.append(p)

        self.pub.publish(self.path)


if __name__ == '__main__':
    rospy.init_node('treasure_hunter')

    target = TargetSelector()
    target.go(5, -3)
    t = Trace()

    rospy.spin()
