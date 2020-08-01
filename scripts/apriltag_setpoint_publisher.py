#!/usr/bin/env python
"""
BSD 3-Clause License

Copyright (c) 2020, Mohamed Abdelkader Zahana
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 """
import rospy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
import tf
from apriltag_ros.msg import AprilTagDetectionArray

class SetpointPublisher:
    def __init__(self):
        self.setpoint_topic_ = rospy.get_param('~setpoint_topic', 'setpoint/relative_pos')

        # Desired altitude above the tag, meters
        self.alt_from_tag_ = rospy.get_param('~alt_from_tag', 1.0)
        # Sanity check. alt_from_tag_ should be non-negative. Otherwise tag will not be seen!
        if self.alt_from_tag_ < 0.0 :
            rospy.logerr("Desired altitude above tag is negative. It should be positive. Defaulting to 1.0 meter")
            self.alt_from_tag_ = 1.0

        # Relative setpoint publisher
        self.setpoint_pub_ = rospy.Publisher(self.setpoint_topic_, Point, queue_size=10)

        # Subscriber to Kalman filter estimate
        rospy.Subscriber("kf/estimate", PoseWithCovarianceStamped, self.kfCallback)

    def kfCallback(self, msg):
        ex = -msg.pose.pose.position.y
        ey = msg.pose.pose.position.x
        ez = msg.pose.pose.position.z + self.alt_from_tag_
        sp_msg = Point()
        sp_msg.x = ex
        sp_msg.y = ey
        sp_msg.z = ez
        self.setpoint_pub_.publish(sp_msg)

if __name__ == '__main__':
    rospy.init_node('tag_setpoint_publisher_node', anonymous=True)

    sp_o = SetpointPublisher()

    rospy.loginfo("tag_setpoint_publisher_node is started")

    rospy.spin()