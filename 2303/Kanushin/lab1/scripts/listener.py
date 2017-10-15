#!/usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import String


class LevelMap:
    def __init__(self, map_file):
        self.map = []
        with open(map_file) as f:
            for line in f:
                tmp = []
                for char in line:
                    tmp.append(char)
                self.map.append(tmp)

    def free(self, x, y):
        return self.map[x][y] == '0'

    def print_map(self, x, y):
        self.map[x][y] = '*'
        res = '\nLevel map:\n'
        for line in self.map:
            res += ''.join(line)
        res += '* - robot\n'
        res += '1 - wall\n'
        res += '0 - empty space\n'
        rospy.loginfo(res)
        self.map[x][y] = '0'


class Robot:
    def __init__(self, name, x=0, y=0):
        self.name = name
        self.x = x
        self.y = y
        self.map = None

    def load_map(self, local_map):
        self.map = local_map

    def move(self, dx, dy):
        new_x = self.x + dx
        new_y = self.y + dy
        if self.map.free(new_x, new_y):
            rospy.loginfo('I am %s, I am moving to (%d,%d)', self.name,
                          new_x, new_y)
            self.x = new_x
            self.y = new_y
        else:
            rospy.loginfo('I am %s, cannot move there', self.name)

    def show_location(self):
        self.map.print_map(self.x, self.y)

    def subscribe(self, topic):
        def callback(data):
            rospy.loginfo('I am %s, my id is %s, I received %s',
                          self.name,
                          rospy.get_caller_id(),
                          data.data)
            if data.data == 'up':
                self.move(-1, 0)

            if data.data == 'left':
                self.move(0, -1)

            if data.data == 'right':
                self.move(0, 1)

            if data.data == 'down':
                self.move(1, 0)

            if data.data == 'print':
                self.show_location()

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber(topic, String, callback)
        rospy.spin()


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    res_path = rospack.get_path('lab_1') + '/res/'
    level = LevelMap('{}map.txt'.format(res_path))

    andy = Robot('Andy', 4, 3)
    andy.load_map(level)
    andy.subscribe('controller')
