from math import sin, cos
from random import randint, choice

import rospy
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import *

from settings import Settings

pi = 3.14


def to_point(x, y):
    return Point(x, y, 0)


def from_point(point):
    return point.x, point.y


class Weapon:
    class Node:
        def __init__(self, prev, x=0, y=0):
            self.direction = 1
            self.direction_count = 100
            self.marker = Marker()
            self.prv = prev
            if self.prv:
                self.prv.nxt = self
            self.nxt = None
            self.marker.type = Marker.ARROW
            self.marker.pose.position = to_point(x, y)
            self.marker.scale.x = 2.0 / randint(2, 5)  # length
            self.marker.scale.y = self.marker.scale.z = 0.1  # thickness
            self.marker.color.r = 1.0 / float(randint(1, 5))
            self.marker.color.g = 1.0 / float(randint(1, 5))
            self.marker.color.b = 1.0 / float(randint(1, 5))
            self.marker.color.a = 1.0

        def add(self):
            return Weapon.Node(self, self.x + self.marker.scale.x, self.y)

        def move_angle(self, angle):
            s = sin(angle)
            c = cos(angle)

            x = self.x
            y = self.y
            x -= self.prv.x
            y -= self.prv.y

            xnew = x * c - y * s
            ynew = x * s + y * c

            finx = xnew + self.prv.x
            finy = ynew + self.prv.y

            self.marker.pose.position = to_point(finx, finy)
            if self.nxt:
                self.nxt.move(finx - x, finy - y)

        def move(self, dx, dy):
            t = self
            while t is not None:
                t.marker.pose.position = to_point(self.x + dx, self.y + dy)
                t = t.nxt

        @property
        def x(self):
            return self.marker.pose.position.x

        @property
        def y(self):
            return self.marker.pose.position.y

        @property
        def last(self):
            t = self
            while t.nxt is not None:
                t = t.nxt

            return t

        def rotate(self, alpha):
            quaternion = (
                self.marker.pose.orientation.x,
                self.marker.pose.orientation.y,
                self.marker.pose.orientation.z,
                self.marker.pose.orientation.w)
            _, _, alpha_orig = euler_from_quaternion(quaternion)
            q = quaternion_from_euler(0, 0, alpha_orig + alpha)
            self.marker.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            if self.nxt:
                self.nxt.move_angle(alpha)

    def __init__(self, robot, nnodes):
        self.robot = robot
        self.root = Weapon.Node(None)
        for i in range(1, nnodes):
            self.root.last.add()

    def rotate(self):
        for node in self.nodes:
            angle = pi / randint(20, 100)

            if node.direction_count < 250:
                node.direction_count += randint(1, 10)
            else:
                node.direction = choice((1, -1))
                node.direction_count = 0

            node.rotate(node.direction * angle)

        self.robot.update()

    @property
    def nodes(self):
        nodes = []
        t = self.root
        while t:
            nodes.append(t)
            t = t.nxt

        return nodes

    def check_hits(self, robots):
        for robot in robots:
            if robot != self.robot:
                if robot.hits(self.global_x, self.global_y):
                    robot.die()

    @property
    def global_x(self):
        return self.root.last.x + self.robot.x

    @property
    def global_y(self):
        return self.root.last.y + self.robot.y

    def markers(self):
        markers = []

        for node in self.nodes:
            markers.append(node.marker)

        return markers


class Robot:
    step = 0.05

    def __init__(self, name, x, y, server):
        self.name = name
        self.server = server
        self.alive = True

        self.controller = None

        self.subscribe(self.id)

        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = Settings.FRAME_ID
        self.int_marker.pose.position = to_point(x, y)
        self.int_marker.scale = 1
        self.int_marker.name = self.id
        self.int_marker.description = name

        self.body = Marker()
        self.body.type = Marker.CUBE
        self.body.scale.x = self.int_marker.scale * 0.5
        self.body.scale.y = self.int_marker.scale * 0.5
        self.body.scale.z = self.int_marker.scale * 0.5
        self.body.color.r = 1.0 / float(randint(1, 5))
        self.body.color.g = 1.0 / float(randint(1, 5))
        self.body.color.b = 1.0 / float(randint(1, 5))
        self.body.color.a = 1.0

        self.weapon = Weapon(self, randint(1, 3))

        self.control = InteractiveMarkerControl()
        self.control.markers.append(self.body)
        self.control.markers += self.weapon.markers()
        self.int_marker.controls.append(self.control)

        server.insert(self.int_marker)
        server.applyChanges()

    def hits(self, x, y):
        if self.x - 0.5 <= x <= self.x + 0.5:
            if self.y - 0.5 <= y <= self.y + 0.5:
                return True

        return False

    def die(self):
        self.controller.stop()

    @property
    def id(self):
        return self.name.replace(' ', '_').lower()

    @property
    def x(self):
        return self.int_marker.pose.position.x

    @property
    def y(self):
        return self.int_marker.pose.position.y

    def move_up(self):
        self.move(self.step, 0)

    def move_down(self):
        self.move(-1 * self.step, 0)

    def move_left(self):
        self.move(0, self.step)

    def move_right(self):
        self.move(0, -1 * self.step)

    def stand_still(self):
        pass

    def move(self, dx, dy):
        x, y = from_point(self.int_marker.pose.position)
        self.int_marker.pose.position = to_point(x + dx, y + dy)
        self.update()

    def update(self):
        self.server.erase(self.int_marker)
        if self.alive:
            self.server.insert(self.int_marker)
        self.server.applyChanges()

    def delete(self):
        self.alive = False

    def subscribe(self, topic):
        def callback(data):
            if data.data == 'up':
                self.move_up()

            if data.data == 'left':
                self.move_left()

            if data.data == 'right':
                self.move_right()

            if data.data == 'down':
                self.move_down()

        rospy.Subscriber(topic, String, callback)
