#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import lab2.srv as srv
from tools import Cmd, Topic, log


class Terminal:
    def __init__(self, ship_id):
        self.name = '{}_terminal'.format(ship_id)
        self.ship = ship_id

        log(self, 'Waiting for {} spaceship to come online'.format(self.ship))
        rospy.wait_for_service('{}_service'.format(self.ship))
        self.repair_proxy = rospy.ServiceProxy('{}_service'.format(self.ship), srv.Repair)

        log(self, 'Subscribing for topic \'{}\''.format(Topic.damaged))

        def inform_damaged(data):
            print '\n ! {} has damaged module: {}'.format(self.ship, data.data)

        rospy.Subscriber(Topic.damaged, String, inform_damaged)

        self.process_input()

    def process_input(self):
        cmd = ''
        while cmd != Cmd.exit:
            args = raw_input('Enter a command\n > ').lower().split(' ')
            cmd = args[0]
            if cmd == Cmd.repair:
                mod_name = ' '.join(args[1:])
                res = self.repair_proxy(mod_name)
                if res.res == 0:
                    print 'Repaired {} successfully'.format(mod_name)
                elif res.res == 1:
                    print 'Module {} is ruined and unable to repair.'.format(mod_name)
                else:
                    print 'Module {} not found'.format(mod_name)
            else:
                if cmd != Cmd.exit:
                    print 'Unknown command'


if __name__ == "__main__":
    rospy.init_node('terminal')
    Terminal('alex')
