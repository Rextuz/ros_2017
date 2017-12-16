#!/usr/bin/env python

import rospy
from pynput.keyboard import Key, Listener, KeyCode, Controller as Kb

from controller import Controller


class ButtonController(Controller):
    KEY_Q = KeyCode(char='q')
    KEY_W = KeyCode(char='w')
    KEY_A = KeyCode(char='a')
    KEY_S = KeyCode(char='s')
    KEY_D = KeyCode(char='d')
    keys = (KEY_Q, KEY_W, KEY_A, KEY_S, KEY_D, Key.esc)

    def __init__(self, topic, sleep):
        Controller.__init__(self, topic, sleep)

        rospy.loginfo('w, a, s, d to move')
        rospy.loginfo('esc, q to exit')

    def run(self):
        Controller.run(self)

        def __on_press(key):
            if key in self.keys:
                if key == self.KEY_W:
                    self.pub.publish('up')
                if key == self.KEY_S:
                    self.pub.publish('down')
                if key == self.KEY_A:
                    self.pub.publish('left')
                if key == self.KEY_D:
                    self.pub.publish('right')

        def __on_release(key):
            if key in (Key.esc, self.KEY_Q):
                return False

        with Listener(on_press=__on_press, on_release=__on_release) as listener:
            listener.join()

    def stop(self):
        keyboard = Kb()

        keyboard.press(self.KEY_Q)
        keyboard.release(self.KEY_Q)
