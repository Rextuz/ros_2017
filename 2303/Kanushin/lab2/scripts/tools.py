import rospy


class Cmd:
    def __init__(self):
        pass

    status = 'status'
    exit = 'exit'
    repair = 'repair'
    print_log = 'log'


class Topic:
    def __init__(self):
        pass

    damaged = 'damaged'
    ruined = 'ruined'


def log(self, msg, level=0):
    final_msg = '{}: {}'.format(self.name, msg)
    if level == 0:
        rospy.loginfo(final_msg)
    if level == 1:
        rospy.logwarn(final_msg)
    if level == 2:
        rospy.logerr(final_msg)
    if level == 3:
        rospy.logfatal(final_msg)
