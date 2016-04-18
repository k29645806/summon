#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def callback_talk(data):
    msg = data.data
    voice = 'voice_kal_diphone'
    soundhandle.say(msg, voice)
    rospy.sleep(1)



if __name__ == '__main__':
    rospy.init_node('talk')
    soundhandle = SoundClient()
    rospy.sleep(1)
    rospy.Subscriber('/talkCmd',String, callback_talk)
    rospy.spin()
