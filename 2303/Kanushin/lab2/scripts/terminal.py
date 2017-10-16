#!/usr/bin/env python

import rospy
from rospy import ServiceException
from std_msgs.msg import String

import lab2.srv as srv
import tools
from tools import Cmd, Topic

log_msgs = []


def log(self, msg, level=0):
    log_msgs.append((self, msg, level))


class Terminal:
    def __init__(self, ship_id):
        self.name = '{}_terminal'.format(ship_id)
        self.ship = ship_id
        self.damaged_modules = []
        self.ruined_modules = []

        log(self, 'Waiting for {} spaceship to come online'.format(self.ship))
        rospy.wait_for_service('{}_service'.format(self.ship))
        self.repair_proxy = rospy.ServiceProxy('{}_service'.format(self.ship), srv.Repair)

        log(self, 'Subscribing for topic \'{}\''.format(Topic.damaged))

        def add_damaged(data):
            log(self, 'Received information about damaged module: {}'.format(data.data))
            if data.data not in self.damaged_modules:
                self.damaged_modules.append(data.data)

        rospy.Subscriber(Topic.damaged, String, add_damaged)

        log(self, 'Subscribing for topic \'{}\''.format(Topic.ruined))

        def add_ruined(data):
            log(self, 'Received information about ruined module: {}'.format(data.data))
            if data.data not in self.ruined_modules:
                self.ruined_modules.append(data.data)
            if data.data in self.damaged_modules:
                self.damaged_modules.remove(data.data)

        rospy.Subscriber(Topic.ruined, String, add_ruined)

        self.process_input()

    def print_status(self):
        if not self.damaged_modules and not self.ruined_modules:
            print 'Everything is fine yet.'

        if self.damaged_modules:
            print 'Damaged modules:'
            for damaged in self.damaged_modules:
                print '\t', damaged

        if self.ruined_modules:
            print 'Ruined modules:'
            for ruined in self.ruined_modules:
                print '\t', ruined

    @staticmethod
    def print_log():
        print '...'
        for slf, msg, level in log_msgs[-5:]:
            tools.log(slf, msg, level)

    def process_input(self):
        cmd = ''
        print 'Available commands:', Cmd.status, Cmd.print_log, Cmd.repair, Cmd.exit
        while cmd != Cmd.exit:
            args = raw_input('Enter a command\n > ').lower().split(' ')
            cmd = args[0]
            if cmd == Cmd.repair:
                module_name = ' '.join(args[1:])

                try:
                    res = self.repair_proxy(module_name)
                except ServiceException:
                    print 'No connection to the repair service. The ship must be dead.'
                    return

                if res.res == 0:
                    print 'Repaired {} successfully'.format(module_name)
                    self.damaged_modules.remove(module_name)
                elif res.res == 1:
                    print 'Module {} is ruined and cannot be repaired.'.format(module_name)
                else:
                    print 'Module {} not found'.format(module_name)
            elif cmd == Cmd.status:
                self.print_status()
            elif cmd == Cmd.print_log:
                self.print_log()
            else:
                if cmd != Cmd.exit:
                    print 'Unknown command'


if __name__ == "__main__":
    rospy.init_node('terminal')
    Terminal('alex')
