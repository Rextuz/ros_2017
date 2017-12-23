#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from rospy import Duration, Publisher as Pub, Subscriber as Sub, Timer
from rospy import init_node, spin


class LocationFinder:
    def __init__(self):
        self.position = None
        Sub('/odom', Odometry, self._callback)

    def _callback(self, odometry):
        self.position = odometry.pose.pose.position


class Trace:
    def __init__(self):
        self.pub = Pub('/trace', Path, queue_size=100)
        self.loc = LocationFinder()
        self.path = Path()
        self.path.header.frame_id = 'odom'
        Timer(Duration.from_sec(1), self._path)

    def _path(self, _):
        p = PoseStamped()
        p.header.frame_id = 'odom'
        p.pose.position = self.loc.position
        self.path.poses.append(p)

        self.pub.publish(self.path)


if __name__ == '__main__':
    init_node(Trace.__name__)

    Trace()

    spin()
