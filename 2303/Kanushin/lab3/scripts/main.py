#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer

from ai_controller import AiController
from button_controller import ButtonController
from robot import Robot
from settings import Settings, duration


def main():
    rospy.init_node(Settings.MARKER_TOPIC)
    server = InteractiveMarkerServer(Settings.MARKER_TOPIC)
    sleep = duration(0.05)

    robots = []
    ctrls = []

    robot0 = Robot('Player', 0, 2, server)
    player = ButtonController(robot0, sleep)
    robots.append(robot0)
    ctrls.append(player)

    robot1 = Robot('Chris', 2, 0, server)
    ai1 = AiController(robot1, sleep)
    robots.append(robot1)
    ctrls.append(ai1)

    robot2 = Robot('Andy', -2, 0, server)
    ai2 = AiController(robot2, sleep)
    robots.append(robot2)
    ctrls.append(ai2)

    robot3 = Robot('Miku', 0, -2, server)
    ai3 = AiController(robot3, sleep)
    robots.append(robot3)
    ctrls.append(ai3)

    def player_alive(_):
        if not player.thread.isAlive():
            for controller in ctrls:
                controller.stop()
            rospy.signal_shutdown('Stopped by user')

    def weapon_rotate(_):
        for robot in robots:
            robot.weapon.rotate()
            robot.weapon.check_hits(robots)

    for ctrl in ctrls:
        ctrl.start()

    rospy.Timer(sleep, player_alive)
    rospy.Timer(sleep, weapon_rotate)
    rospy.spin()


if __name__ == '__main__':
    main()
