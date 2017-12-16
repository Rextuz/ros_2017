#!/usr/bin/env python
from threading import Thread

import rospy
from std_msgs.msg import String


class Controller:
    def __init__(self, robot, sleep):
        self.robot = robot
        self.topic = robot.id
        self.sleep = sleep
        self.thread = None

        robot.controller = self

        self.pub = rospy.Publisher(self.topic, String, queue_size=10)

    def run(self):
        rospy.sleep(self.sleep)

    def start(self):
        self.thread = Thread(target=self.run)
        self.thread.start()

    def stop(self):
        pass
