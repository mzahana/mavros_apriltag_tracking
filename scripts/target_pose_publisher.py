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

class PosePublisher:
  def __init__(self):
    # AprilTag ID to track
    self.tag_id_ = rospy.get_param('~tag_id', 0)

    self.drone_frame_id_ = rospy.get_param('~drone_frame_id', '/base_link')
    self.local_frame_id_ = rospy.get_param('~local_frame_id', '/map')
    self.tag_frame_id_ = rospy.get_param('~tag_frame_id', '/tag_0')
    self.tags_sub_topic_ = rospy.get_param('~tags_topic', 'tag_detections')
    self.target_rel_pose_topic_ = rospy.get_param('~target_rel_pose_topic', 'target/relative_pose')
    self.target_local_pose_topic_ = rospy.get_param('~target_local_pose_topic', 'target/local_pose')

    self.tf_listener_ = tf.TransformListener()

    rospy.Subscriber(self.tags_sub_topic_, AprilTagDetectionArray, self.tagsCallback)

    self.target_rel_pose_pub_ = rospy.Publisher(self.target_rel_pose_topic_, PoseStamped, queue_size=10)
    self.target_local_pose_pub_ = rospy.Publisher(self.target_local_pose_topic_, PoseStamped, queue_size=10)

  def tagsCallback(self, msg):
    valid_rel_pose = False
    valid_local_pose = False
    trans = []
    if len(msg.detections) > 0: # make sure detection is valid
        for tag in msg.detections: 
            if tag.id[0] == self.tag_id_: # deired tag_id
                try: # Relative pose
                    (rel_trans,rel_rot) = self.tf_listener_.lookupTransform(self.drone_frame_id_, self.tag_frame_id_, rospy.Time(0))
                    valid_rel_pose = True
                except :
                    rospy.logwarn("No valid relative pose TF for the required tag %s", self.tag_id_)
                
                try: 
                    (local_trans,local_rot) = self.tf_listener_.lookupTransform(self.local_frame_id_, self.tag_frame_id_, rospy.Time(0))
                    valid_local_pose = True
                except :
                    rospy.logwarn("No valid local pose TF for the required tag %s", self.tag_id_)
                
    if valid_rel_pose: # Publish relative target pose
      pose_msg = PoseStamped()
      pose_msg.header.frame_id = self.drone_frame_id_
      pose_msg.header.stamp = rospy.Time.now()
      pose_msg.pose.position.x = rel_trans[0]
      pose_msg.pose.position.y = rel_trans[1]
      pose_msg.pose.position.z = rel_trans[2]
      pose_msg.pose.orientation.x = rel_rot[0]
      pose_msg.pose.orientation.y = rel_rot[1]
      pose_msg.pose.orientation.z = rel_rot[2]
      pose_msg.pose.orientation.w = rel_rot[3]
      self.target_rel_pose_pub_.publish(pose_msg)

    if valid_local_pose: # Publish local target pose
      pose_msg = PoseStamped()
      pose_msg.header.frame_id = self.local_frame_id_
      pose_msg.header.stamp = rospy.Time.now()
      pose_msg.pose.position.x = local_trans[0]
      pose_msg.pose.position.y = local_trans[1]
      pose_msg.pose.position.z = local_trans[2]
      pose_msg.pose.orientation.x = local_rot[0]
      pose_msg.pose.orientation.y = local_rot[1]
      pose_msg.pose.orientation.z = local_rot[2]
      pose_msg.pose.orientation.w = local_rot[3]
      self.target_local_pose_pub_.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('target_pose_publisher_node', anonymous=True)

    obj = PosePublisher()

    rospy.loginfo("target_pose_publisher_node is started")

    rospy.spin()