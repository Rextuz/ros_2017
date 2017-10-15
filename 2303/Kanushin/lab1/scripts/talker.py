#!/usr/bin/env python

import rospy
from pynput.keyboard import Key, Listener, KeyCode
from std_msgs.msg import String


class KeyPublisher:
    KEY_Q = KeyCode(char='q')
    KEY_W = KeyCode(char='w')
    KEY_A = KeyCode(char='a')
    KEY_S = KeyCode(char='s')
    KEY_D = KeyCode(char='d')
    KEY_P = KeyCode(char='p')
    keys = (KEY_Q, KEY_W, KEY_A, KEY_S, KEY_D, KEY_P, Key.esc)

    def __init__(self, topic):
        self.topic = topic
        self.pub = rospy.Publisher(topic, String, queue_size=10)
        rospy.init_node('key_talker', anonymous=True)

    def start(self):
        def __on_press(key):
            if key in self.keys:
                rospy.loginfo('got key %s', key)
                if key == self.KEY_W:
                    self.pub.publish('up')
                if key == self.KEY_S:
                    self.pub.publish('down')
                if key == self.KEY_A:
                    self.pub.publish('left')
                if key == self.KEY_D:
                    self.pub.publish('right')
                if key == self.KEY_P:
                    self.pub.publish('print')

        def __on_release(key):
            if key in (Key.esc, self.KEY_Q):
                return False

        rospy.loginfo('Controller with topic `%s` launched',
                      self.topic)
        rospy.loginfo('w, a, s, d to move')
        rospy.loginfo('esc, q to exit')
        rospy.loginfo('p to print level map')

        with Listener(on_press=__on_press,
                      on_release=__on_release) as listener:
            listener.join()


if __name__ == '__main__':
    try:
        talker = KeyPublisher('controller')
        talker.start()
    except rospy.ROSInterruptException:
        pass
