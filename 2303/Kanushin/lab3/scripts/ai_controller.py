#!/usr/bin/env python

from random import choice, randint

from rospy import ROSException

from controller import Controller

MIN_MOVES = 20
MAX_MOVES = 50


class AiController(Controller):
    def __init__(self, topic, sleep):
        Controller.__init__(self, topic, sleep)

        self.action = None
        self.repeat = 0
        self.turn = 0
        self.running = True

    def run(self):
        while self.running:
            Controller.run(self)

            if self.turn == self.repeat:
                self.action = choice(('up', 'down', 'left', 'right', None))
                self.repeat = randint(MIN_MOVES, MAX_MOVES)
                self.turn = 0

            if self.action:
                try:
                    self.pub.publish(self.action)
                except ROSException:
                    self.running = False
            self.turn += 1

    def stop(self):
        self.running = False
        self.robot.delete()
