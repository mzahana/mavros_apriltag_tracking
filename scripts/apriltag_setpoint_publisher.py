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
        # AprilTag ID to track
        self.tag_id_ = rospy.get_param('~tag_id', 0)

        self.drone_frame_id_ = rospy.get_param('~drone_frame_id', '/base_link')
        self.tag_frame_id_ = rospy.get_param('~tag_frame_id', '/tag_0')
        self.tags_topic_ = rospy.get_param('~tags_topic', 'tag_detections')
        self.setpoint_topic_ = rospy.get_param('~setpoint_topic', 'setpoint/relative_pos')

        self.tf_listener_ = tf.TransformListener()

        # Desired altitude above the tag, meters
        self.alt_from_tag_ = rospy.get_param('~alt_from_tag', 1.0)
        # Sanity check. alt_from_tag_ should be non-negative. Otherwise tag will not be seen!
        if self.alt_from_tag_ < 0.0 :
            rospy.logerr("Desired altitude above tag is negative. It should be positive. Defaulting to 1.0 meter")
            self.alt_from_tag_ = 1.0

        # Relative setpoint publisher
        self.setpoint_pub_ = rospy.Publisher(self.setpoint_topic_, Point, queue_size=10)

        # Relative pose publisher as PoseStamped
        self.pose_pub_ = rospy.Publisher('tag/pose', PoseStamped, queue_size=10)

        # Subscriber to Kalman filter estimate
        rospy.Subscriber("kf/estimate", PoseWithCovarianceStamped, self.kfCallback)

        rospy.Subscriber(self.tags_topic_, AprilTagDetectionArray, self.tagsCallback)

    def kfCallback(self, msg):
        ex = msg.pose.pose.position.x
        ey = msg.pose.pose.position.y
        ez = msg.pose.pose.position.z + self.alt_from_tag_
        sp_msg = Point()
        sp_msg.x = ex
        sp_msg.y = ey
        sp_msg.z = ez
        self.setpoint_pub_.publish(sp_msg)

    # tags callback
    def tagsCallback(self, msg):
        valid = False
        trans = []
        if len(msg.detections) > 0: # make sure detection is valid
            for tag in msg.detections: 
                if tag.id[0] == self.tag_id_: # deired tag_id
                    try:
                        (trans,rot) = self.tf_listener_.lookupTransform(self.drone_frame_id_, self.tag_frame_id_, rospy.Time(0))
                        valid = True
                    except :
                        rospy.logwarn("No valid TF for the required tag %s", self.tag_id_)
                        return
        if valid: # Publish relative setpoint
            # for debug
            #rospy.loginfo("Tag %s is x=%s , y=%s , z =%s away from the drone", self.tag_id_, trans[0], trans[1], trans[2])
            
            # compute error in drone frame
            # The one we get in trans is x-forward, y-left, z-up
            # the one we should send is x-right, y-forward, z-up
            # ex = -trans[1]
            # ey = trans[0]
            # ez = trans[2] + self.alt_from_tag_
            # sp_msg = Point()
            # sp_msg.x = ex
            # sp_msg.y = ey
            # sp_msg.z = ez
            # self.setpoint_pub_.publish(sp_msg)

            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.drone_frame_id_
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]
            pose_msg.pose.orientation.x = rot[0]
            pose_msg.pose.orientation.y = rot[1]
            pose_msg.pose.orientation.z = rot[2]
            pose_msg.pose.orientation.w = rot[3]
            self.pose_pub_.publish(pose_msg)
        else: 
            pass
                


if __name__ == '__main__':
    rospy.init_node('tag_setpoint_publisher_node', anonymous=True)

    sp_o = SetpointPublisher()

    rospy.loginfo("tag_setpoint_publisher_node is started")

    rospy.spin()