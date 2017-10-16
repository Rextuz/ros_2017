#!/usr/bin/env python

import random
import signal
from threading import Thread
from time import sleep

import rospy
from std_msgs.msg import String

import lab2.srv as srv
from tools import log, Topic

TICK = 5


class Module:
    hp_threshold = 80
    hp_max = 100
    msg_frequency = 10

    def __init__(self, name):
        self.name = name.lower()
        self._hp = self.hp_max
        self.sending_sos = False
        log(self, 'Successful init')

        self.pub_damaged = rospy.Publisher(Topic.damaged, String, queue_size=32)
        self.pub_ruined = rospy.Publisher(Topic.ruined, String, queue_size=32)

    @property
    def hp(self):
        return self._hp

    def is_working(self):
        return self._hp > 0

    def take_damage(self, damage):
        self._hp -= damage

        # if no hp left the module dies and stops sending signals
        if self.hp < 0:
            self._hp = 0
            self.sending_sos = False
            log(self, 'Module is ruined', level=2)
            self.pub_ruined.publish(self.name)
            return

        log(self, 'Took {} damage. {} hp.'.format(damage, self.hp), level=1)
        if self._hp < self.hp_threshold:
            if not self.sending_sos:
                self.ask_for_help()

    def ask_for_help(self):
        def sos_signal():
            self.sending_sos = True
            while self.hp < self.hp_threshold and self.sending_sos:
                log(self, 'Asking for help.')
                self.pub_damaged.publish(self.name)
                sleep(self.msg_frequency)

        Thread(target=sos_signal).start()

    def repair(self):
        self._hp = self.hp_max

    def halt(self):
        self.sending_sos = False


class Spaceship:
    n_modules_critical = 3

    def __init__(self, name):
        self.name = name
        self._modules = []

        def repair(req):
            mod_to_fix = req.name.lower()
            log(self, 'Repairing {}'.format(mod_to_fix))

            for mod in self._modules:
                if mod.name.lower() == mod_to_fix:
                    if mod.hp > 0:
                        mod.repair()
                        return srv.RepairResponse(0)

                    return srv.RepairResponse(1)

            return srv.RepairResponse(-1)

        self.repair_service = rospy.Service('{}_service'.format(self.id), srv.Repair, repair)
        log(self, 'Repair service is online and active.')

    @property
    def id(self):
        return self.name.lower().replace(' ', '_')

    def add_module(self, new_module):
        self._modules.append(new_module)

    def start(self):
        log(self, 'Starting')
        while self.can_fly():
            sleep(TICK)
            damaged = random.choice([True, False])
            if damaged:
                damage_received = random.randint(10, 50)
                log(self, 'Received {} damage'.format(damage_received), level=1)
                module_damaged = random.choice(self._modules)
                module_damaged.take_damage(damage_received)

        log(self, 'Critical damage.', level=2)
        self.halt()

    def halt(self, sig=None, frame=None):
        if sig or frame:
            log(self, 'Terminated by SIGINT.')
            exit(0)

        log(self, 'Shutting down.')
        for mod in self._modules:
            mod.halt()

    def can_fly(self):
        dead_modules = [m for m in self._modules if not m.is_working()]
        if dead_modules:
            log(self, '{} dead modules'.format(len(dead_modules)))
        return len(dead_modules) < self.n_modules_critical


if __name__ == "__main__":
    rospy.init_node('spaceship')
    spaceship = Spaceship('Alex')
    spaceship.add_module(Module('Left turret'))
    spaceship.add_module(Module('Right turret'))
    spaceship.add_module(Module('Engine'))
    spaceship.add_module(Module('Cockpit'))
    spaceship.add_module(Module('Thruster'))
    signal.signal(signal.SIGINT, spaceship.halt)
    spaceship.start()
