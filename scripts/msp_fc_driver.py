#! /usr/bin/python
import msp
from msp import MSP
import rospy
import struct
import math
from mav_msgs.msg import RateThrust
from std_msgs.msg import Bool


class MspDriver:
    def __init__(self):
        self.msp = MSP()
        self.max_roll_r = 673
        self.max_pitch_r = 673
        self.max_yaw_r = 673
        self.rcData = [1500, 1500, 900, 1500, 0]

    #def send_rc_tuning(self):
     #   if not self.

    def send_rates(self, rates):
        roll_r    = rates.angular_rates.x * 360 / math.pi / self.max_roll_r
        pitch_r   = rates.angular_rates.y * 360 / math.pi / self.max_pitch_r
        yaw_r     = rates.angular_rates.z * 360 / math.pi / self.max_yaw_r
        self.rcData[0] = min(500, max(-500, round(roll_r  * 500))) + 1500
        self.rcData[1] = min(500, max(-500, round(pitch_r * 500))) + 1500
        self.rcData[2] = min(500, max(-500, round(yaw_r   * 500))) + 1500
        self.rcData[3] = min(1000, max(0, round(rates.thrust.z * 1000))) + 1000

    def arm(self, arm_value):
        if arm_value.data:
            self.rcData[4] = 2000
        else:
            self.rcData[4] = 1000

    def get_rc_cmd(self, payload):
        n = len(payload) / 2
        rcData = struct.unpack("<{}H".format(n), payload)
        rospy.loginfo("rcData: %s", str(rcData))

    def run(self):
        self.msp.register_callback(MSP.RC, self.get_rc_cmd)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.msp.send_message(MSP.RC, b'')
            self.msp.send_message(MSP.SET_RAW_RC, struct.pack("<5H", *self.rcData))
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("msp_fc_driver")

    driver = MspDriver()

    rospy.Subscriber("/uav/control/rate_thrust", RateThrust, driver.send_rates)
    rospy.Subscriber("/uav/control/arm", Bool, driver.arm)
    driver.run()
