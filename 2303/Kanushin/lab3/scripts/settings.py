import rospy


class Settings:
    FRAME_ID = 'map'
    MARKER_TOPIC = 'robots'

    def __init__(self):
        pass


def duration(sec):
    return rospy.Duration(0, sec * 10e8)
