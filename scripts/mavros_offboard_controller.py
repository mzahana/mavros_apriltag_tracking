#!/usr/bin/env python

import rospy
import numpy as np
import tf

from math import pi, sqrt
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Vector3, Point, PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

class FCUModes:
    def __init__(self):
	    pass    

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set."%e
##########################################################################################
"""
This is a PI controller which takes target positions (ex_, ey_, ez_) in body directions (i.e. relative to body).
It ouputs velocity commands (body_vx_cmd_, body_vx_cmd_, body_vz_cmd_) in body directions.
"""
class PositionController:
    def __init__(self):
        # Error in body x direction (+x is drone's right)
        self.ex_ = 0.0
        # Error in body y direction (+y is  drone's front)
        self.ey_ = 0.0
        # Error in body z direction (+z is  up)
        self.ez_ = 0.0
        # Error integral in x
        self.ex_int_ = 0.0
        # Error integral in y
        self.ey_int_ = 0.0
        # Error integral in z
        self.ez_int_ = 0.0

        # Proportional gain for horizontal controller
        self.kP_xy_ = rospy.get_param('~horizontal_controller/kP', 1.5)
        # Integral gain for horizontal controller
        self.kI_xy_ = rospy.get_param('~horizontal_controller/kI', 0.01)
        # Integral gain for vertical controller
        self.kP_z_ = rospy.get_param('~vertical_controller/kP', 2.0)
        # Integral gain for vertical controller
        self.kI_z_ = rospy.get_param('~vertical_controller/kI', 0.01)

        # Controller outputs. Velocity commands in body sudo-frame
        self.body_vx_cmd_ = 0.0
        self.body_vy_cmd_ = 0.0
        self.body_vz_cmd_ = 0.0

        # Controller outputs in local frame
        self.local_vx_cmd_ = 0.0
        self.local_vy_cmd_ = 0.0
        self.local_vz_cmd_ = 0.0
        
        # Maximum horizontal velocity (m/s)
        self.vXYMAX_ = rospy.get_param('~horizontal_controller/vMAX', 1.0)
        # Maximum upward velocity (m/s)
        self.vUpMAX_ = rospy.get_param('~vertical_controller/vUpMAX', 1.0)
        # Maximum downward velocity (m/s)
        self.vDownMAX_ = rospy.get_param('~vertical_controller/vDownMAX', 0.5)

        # Flag for FCU state. True if vehicle is armed and ready
        # Prevents building controller integral part when vehicle is idle on ground
        self.engaged_ = False

        # Subscribe to drone FCU state
        rospy.Subscriber('mavros/state', State, self.cbFCUstate)

    def cbFCUstate(self, msg):
        if msg.armed and msg.mode == 'OFFBOARD' :
            self.engaged_ = True
        else:
            self.engaged_ = False

    def resetIntegrators(self):
        self.ex_int_ = 0.
        self.ey_int_ = 0.
        self.ez_int_ = 0.

    def computeVelSetpoint(self):
        """
        Computes XYZ velocity setpoint in body sudo-frame using a PI controller
        """
        # Compute commands
        self.body_vx_cmd_ = self.kP_xy_*self.ex_ + self.kI_xy_*self.ex_int_
        self.body_vy_cmd_ = self.kP_xy_*self.ey_ + self.kI_xy_*self.ey_int_
        self.body_vz_cmd_ = self.kP_z_*self.ez_ + self.kI_z_*self.ez_int_

        # Horizontal velocity constraints
        vel_magnitude = sqrt(self.body_vx_cmd_**2 + self.body_vy_cmd_**2)
        if vel_magnitude > self.vXYMAX_ : # anti-windup scaling      
            scale = self.vXYMAX_/vel_magnitude
            self.body_vx_cmd_ = self.body_vx_cmd_*scale
            self.body_vy_cmd_ = self.body_vy_cmd_*scale
        else:
            if self.engaged_: # if armed & offboard
                self.ex_int_ = self.ex_int_ + self.ex_ # You can divide self.ex_ by the controller rate, but you can just tune self.kI_xy_ for now!
                self.ey_int_ = self.ey_int_ + self.ey_

        # Vertical velocity constraints
        if self.body_vz_cmd_ > self.vUpMAX_ : # anti-windup scaling      
            self.body_vz_cmd_ = self.vUpMAX_
        elif self.body_vz_cmd_ < -self.vDownMAX_:
            self.body_vz_cmd_ = -self.vDownMAX_
        else:
            if self.engaged_: # if armed & offboard
                self.ez_int_ = self.ez_int_ + self.ez_ # You can divide self.ex_ by the controller rate, but you can just tune self.kI_z_ for now!

        return self.body_vx_cmd_, self.body_vy_cmd_, self.body_vz_cmd_

##########################################################################################

class Commander:
    def __init__(self):
        # Instantiate a setpoint topic structure
        self.setpoint_ = PositionTarget()

        # use velocity and yaw setpoints
        self.setBodyVelMask()

        # Velocity setpoint by user
        self.vel_setpoint_ = Vector3()

        # Position setpoint by user
        self.pos_setpoint_ = Point()

        # Yaw setpoint by user (degrees); will be converted to radians before it's published
        self.yaw_setpoint_ = 0.0

        # Current drone position
        self.drone_pos_ = Point()

        # FCU modes
        self.fcu_mode_ = FCUModes()

        # setpoint publisher (velocity to Pixhawk)
        self.setpoint_pub_ = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # Subscriber for user setpoints (body velocity)
        #rospy.Subscriber('setpoint/body_vel', Vector3, self.velSpCallback)

        # Subscriber for user setpoints (local position)
        #rospy.Subscriber('setpoint/local_pos', Point, self.posSpCallback)

        # Subscriber for user setpoints (yaw in degrees)
        rospy.Subscriber('setpoint/yaw_deg', Float32, self.yawSpCallback)

        # Subscriber to current drone's position
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)

        # Service for arming and setting OFFBOARD flight mode
        rospy.Service('arm_and_offboard', Empty, self.armAndOffboard)

        # Service for autoland
        rospy.Service('auto_land', Empty, self.autoLand)

        # Service for holding at current position
        # rospy.Service('hold', Empty, self.hold)

    # def hold(self, req):
    #     self.pos_setpoint_.x = self.drone_pos_.x
    #     self.pos_setpoint_.y = self.drone_pos_.y
    #     self.pos_setpoint_.z = self.drone_pos_.z

    #     self.setLocalPositionMask()
    #     return EmptyResponse()

    def autoLand(self, req):
        self.fcu_mode_.setAutoLandMode()

        return EmptyResponse()

    def armAndOffboard(self, req):
        self.fcu_mode_.setArm()
        self.fcu_mode_.setOffboardMode()
        
        return EmptyResponse()

    def dronePosCallback(self, msg):
        self.drone_pos_.x = msg.pose.position.x
        self.drone_pos_.y = msg.pose.position.y
        self.drone_pos_.z = msg.pose.position.z

    def velSpCallback(self, msg):
        """
        Velocity setpoint callback
        """
        self.vel_setpoint_.x = msg.x
        self.vel_setpoint_.y = msg.y
        self.vel_setpoint_.z = msg.z

        self.setBodyVelMask()
    
    def posSpCallback(self, msg):
        """
        Position setpoint callback
        """
        self.pos_setpoint_.x = msg.x
        self.pos_setpoint_.y = msg.y
        self.pos_setpoint_.z = msg.z

        self.setLocalPositionMask()

    def yawSpCallback(self, msg):
        """
        Yaw setpoint callback
        """
        self.yaw_setpoint_ = msg.data


    def setLocalPositionMask(self):
        """
        Sets type_mask for position setpoint in local frame +  yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE

    def setBodyVelMask(self):
        """
        Sets type_mask for velocity setpoint in body frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE

    def setLocalVelMask(self):
        """
        Sets type_mask for velocity setpoint in local frame + yaw setpoint
        """
        # FRAME_LOCAL_NED, FRAME_LOCAL_OFFSET_NED, FRAME_BODY_NED, FRAME_BODY_OFFSET_NED
        self.setpoint_.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + \
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + \
                                    PositionTarget.IGNORE_YAW_RATE

    def publishSetpoint(self):
        self.setpoint_.header.stamp = rospy.Time.now()

        # Only one type of the following setpoints will be consumed based on the type_mask
        self.setpoint_.position.x = self.pos_setpoint_.x
        self.setpoint_.position.y = self.pos_setpoint_.y
        self.setpoint_.position.z = self.pos_setpoint_.z

        self.setpoint_.velocity.x = self.vel_setpoint_.x
        self.setpoint_.velocity.y = self.vel_setpoint_.y
        self.setpoint_.velocity.z = self.vel_setpoint_.z

        self.setpoint_.yaw = self.yaw_setpoint_ * pi / 180. # convert to radians

        self.setpoint_pub_.publish(self.setpoint_)

#####################################################################################
"""
Provides methods to track a point in both body and local frames.
It uses PositionController class to compute control signals and Commander class to send them to the drone FCU

Set self.local_tracking_ = True to track points in local frame, and False to track in body relative frame
"""
class Tracker:
    def __init__(self):
        # setpoints in local frame
        self.local_xSp_  = 0.0
        self.local_ySp_  = 0.0
        self.local_zSp_  = 0.0

        # Relative setpoints (i.e. with respect to body horizontal-frame)
        self.relative_xSp_  = 0.0
        self.relative_ySp_  = 0.0
        self.relative_zSp_  = 0.0

        # Flag to select between local vs. relative tracking
        # set False for relative target tracking
        self.local_tracking_ = None

        # Commander object
        self.commander_ = Commander()

        # Controller object
        self.controller_ = PositionController()

        # Subscriber for user setpoints (local position)
        rospy.Subscriber('setpoint/local_pos', Point, self.localPosSpCallback)

        # Subscriber for user setpoints (relative position)
        rospy.Subscriber('setpoint/relative_pos', Point, self.relativePosSpCallback)

    def computeControlOutput(self):
        if self.local_tracking_:
            self.controller_.ex_ = self.local_xSp_ - self.commander_.drone_pos_.x
            self.controller_.ey_ = self.local_ySp_ - self.commander_.drone_pos_.y
            self.controller_.ez_ = self.local_zSp_ - self.commander_.drone_pos_.z
            self.commander_.setLocalVelMask()
        else: # relative tracking
            self.controller_.ex_ = self.relative_xSp_
            self.controller_.ey_ = self.relative_ySp_
            self.controller_.ez_ = self.relative_zSp_
            self.commander_.setBodyVelMask()

        self.commander_.vel_setpoint_.x, self.commander_.vel_setpoint_.y, self.commander_.vel_setpoint_.z = self.controller_.computeVelSetpoint()

    def localPosSpCallback(self, msg):
        self.local_xSp_ = msg.x
        self.local_ySp_ = msg.y
        self.local_zSp_ = msg.z

        # In case we are switching from relative to local tracking
        # to avoid jumps caused by accumulation in the integrators
        if not self.local_tracking_ or self.local_tracking_ is None:
            self.controller_.resetIntegrators()
            self.local_tracking_ = True

    def relativePosSpCallback(self, msg):
        self.relative_xSp_ = msg.x
        self.relative_ySp_ = msg.y
        self.relative_zSp_ = msg.z

        # In case we are switching from local to relative tracking
        # to avoid jumps caused by accumulation in the integrators
        if self.local_tracking_ or self.local_tracking_ is None:
            self.controller_.resetIntegrators()
            self.local_tracking_ = False

if __name__ == '__main__':
    rospy.init_node('Offboard_control_node', anonymous=True)
    
    tracker = Tracker()

    loop = rospy.Rate(20)

    while not rospy.is_shutdown():

        """
        # Example of how to use the PositionController

        K = PositionController() # Should be created outside ROS while loop
        
        # The following should be inside ROS while loop
        # update errors
        K.ex_ = relative_position_in_x # body directions
        K.ey_ = relative_position_in_y
        K.ez_ = relative_position_in_z

        cmd.vel_setpoint_.x, cmd.vel_setpoint_.y, cmd.vel_setpoint_.z = K.computeVelSetpoint()
        cmd.setBodyVelMask()
        # Then, publish command as below (publishSetpoint)

        """
        tracker.computeControlOutput()
        tracker.commander_.publishSetpoint()
        #cmd.publishSetpoint()
        loop.sleep()