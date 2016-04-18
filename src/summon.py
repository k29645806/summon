#!/usr/bin/env python
import rospy
import time
from math import degrees
from numpy import median, std, mean
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32,Twist
from std_msgs.msg import String
from kobuki_msgs.msg import BumperEvent,Led
import tf
from tf.transformations import euler_from_quaternion

class Kobuki(object):
    fwd_speed, rotate_speed = 0.0, 0.0
    x, y, heading = 0.0, 0.0, 0.0  ## heading unit: degrees
    x_record, y_record, heading_record, final_voice_angle_record = 0.0, 0.0, 0.0, 0.0
    voice_angle = -1   ## voice angle unit: degrees
    voiceID = 0
    heading_desire, distance_desire = 0.0, 2.0
    state = 'idle'
    angle_remain, distance_remain = 0.0, 0.0

    def __init__(self, fwd_speed=100, rotate_speed=100):
        self.default_fwd_speed = fwd_speed
        self.default_rotate_speed = rotate_speed
    def subscribe(self):
        rospy.Subscriber('/odom',Odometry, self.callback_odom)  # from robot_setup.py
        rospy.Subscriber("/serial/read", String, self.callback_voice_angle) # from sound_localization.py
        rospy.Subscriber('/output',String, self.callback_voice_cmd)  # from recognizer.py
    def callback_odom(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        q1 = data.pose.pose.orientation.x
        q2 = data.pose.pose.orientation.y
        q3 = data.pose.pose.orientation.z
        q4 = data.pose.pose.orientation.w
        q = (q1, q2, q3, q4)
        e = euler_from_quaternion(q)
        yaw = degrees(e[2])
        self.heading = self.to_positive_angle(yaw)
    def callback_voice_angle(self, data):
        self.voice_angle = -1 if int(data.data)<0 else -int(data.data) + 360  # cvt from CW to CCW
    def callback_voice_cmd(self, data):
        voice_cmd = data.data
        if voice_cmd=="come here":
            rospy.loginfo("'Come here' command received!")
            for i in range(20):
                self.voiceID = 1
                time.sleep(0.01)
            self.voiceID = 0   # finish trigger
        elif voice_cmd=="stop":
            rospy.loginfo("'stop' command received!")
            for i in range(20):
                self.voiceID = 2
                time.sleep(0.01)
        else:
            rospy.loginfo("Invaild command")
            self.voiceID = 0

    def idle(self, final_voice_angle, exist=False):
        self.fwd_speed = 0.0
        self.rotate_speed = 0.0
        self.x_record = self.x
        self.y_record = self.y
        self.heading_record = self.heading
        self.final_voice_angle_record = final_voice_angle
        bolin = exist and self.voiceID==1
        rospy.loginfo("voice angle:%s voiceID:%s start:%s"%(self.final_voice_angle, self.voiceID, bolin))
        #self.state = 'rotate' if (exist and self.voiceID==1) else 'idle'
    def rotate(self):
        self.fwd_speed = 0.0
        self.rotate_speed = self.default_rotate_speed
        self.heading_desire = self.heading_record + self.final_voice_angle_record
        self.angle_remain = self.find_smallest_rotation_angle(self.heading_desire, self.heading)
        self.rotate_speed = self.default_rotate_speed if self.angle_remain > 0 else -self.default_rotate_speed
    def rotate_state_check(self):
        if abs(self.angle_remain) <= 5.0:
            rospy.loginfo('Reached desired heading')
            self.state = 'forward'
        else:
            self.state = 'rotate'
    def forward(self):
        self.fwd_speed = self.default_fwd_speed
        self.rotate_speed = 0.0
        dx = self.x - self.x_record
        dy = self.y - self.y_record
        self.distance_remain = (dx**2 + dy**2)**0.5
    def forward_state_check(self):
        if self.distance_remain >= self.distance_desire:
            rospy.loginfo('Reached')
            self.state = 'idle'
        else:
            self.state = 'forward'
## ----------------------------------------------------------------------------
    def print_info(self, possibility):
        #print 'state:%s x:%s y:%s heading:%s'%(self.state, self.x, self.y, self.heading)
        #print 'state:%s possibility:%s'% (self.state, possibility)
        #print 'voice_angle:%s angle_remain:%s distance_remain:%s'\
        #%(self.final_voice_angle_record, self.angle_remain, self.distance_remain)
        #print self.voiceID
    def pub_cmd(self):
        pub = rospy.Publisher('/cmd_vel' ,Twist, queue_size=10)
        msg = Twist()
        msg.linear.x = self.fwd_speed
        msg.angular.z = self.rotate_speed
        pub.publish(msg)
    def find_smallest_rotation_angle(self, heading_desire, heading_now):
        heading_desire = heading_desire % 360
        heading_now = heading_now % 360
        a = heading_desire - heading_now
        b = (heading_desire + 360) - heading_now
        c = heading_desire - (heading_now + 360)
        lst = [a, b, c]
        lst_abs = [abs(x) for x in lst]
        index = lst_abs.index(min(lst_abs))
        return lst[index]
    def to_positive_angle(self, theta):
        while True:
            if theta < 0:
                theta += 360
            if theta > 0:
                theta = theta % 360
                return theta
                break

class Voice(object):
    def __init__(self, freq_mcu=30.0, freq_main=100.0 , voice_duration=1):
        self.freq_mcu = freq_mcu
        self.freq_main = freq_main
        self.max_percentage = freq_mcu / freq_main
        self.check_length = int(freq_main * voice_duration)
        self.voice_angle_lst = [-1] * self.check_length

    def siso(self, lst, value):
        lst.insert(0, value)
        lst.pop()
        return lst

    def listen(self, voice_angle):
        self.voice_angle_lst = self.siso(self.voice_angle_lst, voice_angle)
        signal_numbers = self.check_length - self.voice_angle_lst.count(-1) ##    Signal in a moving array

        if signal_numbers:
            heard_lst = [x for x in self.voice_angle_lst if x >=0]
            self.std = std(heard_lst) / mean(heard_lst)
            self.percentage = float(signal_numbers) / self.check_length
            self.possibility = self.percentage / self.max_percentage
        else:
            self.possibility = 0.0

        if self.possibility > 0.6:
            self.exist = True
            self.final_voice_angle = median(heard_lst)
            self.temp = self.voice_angle_lst
            self.voice_angle_lst = [-1] * self.check_length   ## Reset the moving array
        else:
            self.exist = False
            self.final_voice_angle = 0


rospy.init_node("summon")
rate = rospy.Rate(100)
ear = Voice()
kbk = Kobuki()
kbk.subscribe()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        ear.listen(kbk.voice_angle)
        if kbk.state == 'idle':
            kbk.idle(ear.final_voice_angle, ear.exist)
        elif kbk.state == 'rotate':
            kbk.rotate()
            kbk.rotate_state_check()
        elif kbk.state == 'forward':
            kbk.forward()
            kbk.forward_state_check()

        kbk.pub_cmd()
        rate.sleep()
