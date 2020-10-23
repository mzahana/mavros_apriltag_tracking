/*
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
*/

#include "mavros_apriltag_tracking/kf_tracker.h"

KFTracker::KFTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
nh_(nh),
nh_private_(nh_private),
dt_pred_(0.05),// 20Hz
q_(0.1),
r_(0.01),
is_state_initialzed_(false),
state_buffer_size_(40),
tracking_frame_("base_link"),
do_update_step_(true),
measurement_off_time_(2.0),
debug_(false)
{
   nh_private_.param<double>("dt_pred", dt_pred_, 0.05);
   nh_private_.param<double>("q_std", q_, 0.1);
   nh_private_.param<double>("r_std", r_, 0.01);
   int buff_size;
   nh_private_.param<int>("state_buffer_length", buff_size, 40);
   state_buffer_size_  = (unsigned int) buff_size;
   ROS_INFO("State buffer length corresponds to %f seconds", dt_pred_*(double)state_buffer_size_);
   nh_private.param<std::string>("tracking_frame_id",tracking_frame_ , "base_link");
   nh_private.param<bool>("do_kf_update_step", do_update_step_, true);
   nh_private.param<double>("measurement_off_time", measurement_off_time_, 2.0);
   nh_private.param<bool>("print_debug_msg", debug_, false);

   initKF();

   kf_loop_timer_ =  nh_.createTimer(ros::Duration(dt_pred_), &KFTracker::filterLoop, this); // Define timer for constant loop rate

   pose_sub_ =  nh_.subscribe("measurement/pose", 1, &KFTracker::poseCallback, this);

   state_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("kf/estimate", 1);
   
}

KFTracker::~KFTracker()
{
}

void KFTracker::setQ(void)
{
   // This is for constant velocity model \in R^3

   Q_.resize(6,6);
   Q_ = Eigen::MatrixXd::Zero(6,6);
   Q_(0,0) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // x with x
   Q_(0,3) = 0.5*dt_pred_*dt_pred_; // x with vx
   Q_(1,1) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // y with y
   Q_(1,4) = 0.5*dt_pred_*dt_pred_; // y with vy
   Q_(2,2) = 1./3.*dt_pred_*dt_pred_*dt_pred_; // z with z
   Q_(2,5) = 0.5*dt_pred_*dt_pred_; // z with vz
   Q_(3,0) = Q_(0,3); // vx with x. Symmetric
   Q_(3, 3) = dt_pred_; // vx with vx
   Q_(4,1) = Q_(1,4); // vy with y. Symmetric
   Q_(4,4) = dt_pred_; // vy with vy
   Q_(5,2) = Q_(2,5); // vz with z. Symmetric
   Q_(5,5) = dt_pred_; // vz with vz

   Q_ = q_*q_*Q_; // multiply by process noise variance
}

void KFTracker::setR(void)
{
   R_.resize(3,3);
   R_ = Eigen::MatrixXd::Identity(3,3);
   R_ = r_*r_*R_; // multiply by observation noise variance
}

void KFTracker::setF(void)
{
   // This is for constant velocity model \in R^3

   F_.resize(6,6);
   F_ = Eigen::MatrixXd::Identity(6,6);
   F_(0,3) = dt_pred_; // x - vx
   F_(1,4) = dt_pred_; // y - vy
   F_(2,5) = dt_pred_; // z - vz
}

void KFTracker::setH(void)
{
   H_.resize(3,6);
   H_ = Eigen::MatrixXd::Zero(3,6);
   H_(0,0) = 1.0; // observing x
   H_(1,1) = 1.0; // observing y
   H_(2,2) = 1.0; // observing z
}

void KFTracker::initP(void)
{
   kf_state_pred_.P = Q_;
}

void KFTracker::initKF(void)
{
   // initial state KF estimate
   kf_state_pred_.time_stamp = ros::Time::now();
   kf_state_pred_.x = Eigen::MatrixXd::Zero(6,1); // position and velocity \in R^3
   kf_state_pred_.x(3,0) = 0.0000001;
   kf_state_pred_.x(4,0) = 0.0000001;
   kf_state_pred_.x(5,0) = 0.0000001;

   setQ(); // Initialize process noise covariance
   initP();
   setR(); // Initialize observation noise covariance
   setF(); // Initialize transition matrix
   setH(); // Initialize observation matrix

   state_buffer_.clear(); // Clear state buffer

   z_meas_.time_stamp = ros::Time::now();
   z_meas_.z.resize(3,1); 
   z_meas_.z = Eigen::MatrixXd::Zero(3,1); //  measured position \in R^3
   z_last_meas_ = z_meas_;


   ROS_INFO("KF is initialized.");
}

void KFTracker::updateStateBuffer(void)
{
   kf_state kfstate;
   kfstate.time_stamp = kf_state_pred_.time_stamp;
   kfstate.x = kf_state_pred_.x;
   kfstate.P = kf_state_pred_.P;
   state_buffer_.push_back(kfstate);
   if(state_buffer_.size() > state_buffer_size_)
      state_buffer_.erase(state_buffer_.begin()); // remove first element in the buffer
}

bool KFTracker::predict(void)
{
   kf_state_pred_.x = F_*kf_state_pred_.x;
   if(kf_state_pred_.x.norm() > 10000.0)
   {
      ROS_ERROR("state prediciotn exploded!!!");
      is_state_initialzed_ = false;
      initKF();
      return false;
   }
   kf_state_pred_.P = F_*kf_state_pred_.P*F_.transpose() + Q_;
   kf_state_pred_.time_stamp = ros::Time::now();
   
   // Add state to buffer
   updateStateBuffer();

   return true;
}

void KFTracker::update(void)
{
   sensor_measurement current_z; // store the current measurement in seperate variable as the original one is continuously updated
   current_z.time_stamp = z_meas_.time_stamp;
   current_z.z = z_meas_.z;

   // check if we got new measurement
   if (current_z.time_stamp.toSec() == z_last_meas_.time_stamp.toSec())
   {
      if(debug_)
         ROS_WARN("No new measurment. Skipping KF update step.");
      return;
   }

   // Make sure the current measurement time is not ahead of current prediction
   if(current_z.time_stamp.toSec() > state_buffer_.back().time_stamp.toSec())
   {;
      if(debug_)
         ROS_WARN("Measurement is ahead of prediction. Skipping KF update step.");
      return;
   }

   bool state_found = false;
   if(state_buffer_.size() > 1)
   {
      // Find closest prediction time to the current measurement. Then, use it for update.
      //for (std::vector<kf_state>::iterator it = state_buffer_.end() ; it != state_buffer_.begin(); it--)
      for (int i=0; i < state_buffer_.size(); i++)
      {
         auto dt = state_buffer_[i].time_stamp.toSec() - current_z.time_stamp.toSec();
         if( dt > dt_pred_ and dt < 2*dt_pred_)
         {
            kf_state_pred_.time_stamp = state_buffer_[i].time_stamp;
            kf_state_pred_.x = state_buffer_[i].x;
            kf_state_pred_.P = state_buffer_[i].P;
            state_buffer_.erase(state_buffer_.begin(),state_buffer_.begin()+i+1); // remove old states
            state_found = true;
            break;
         }
      } // end loop over state_buffer_
   }

   if(state_found)//do KF update step
   {
      // Following Wikipedia KF convention

      // compute innovation
      auto y = current_z.z - H_*kf_state_pred_.x;

      // Innovation covariance
      auto S = H_*kf_state_pred_.P*H_.transpose() + R_;

      // Kalman gain
      auto K = kf_state_pred_.P*H_.transpose()*S.inverse();

      // Updated state estimate and its covariance
      kf_state_pred_.x = kf_state_pred_.x + K*y;
      kf_state_pred_.P = kf_state_pred_.P - K*H_*kf_state_pred_.P;

      // For debugging
      if(debug_)
         ROS_INFO("KF update step is executed.");

      // z_last_meas_.time_stamp = z.time_stamp;
      // z_last_meas_.z = z.z;
      z_last_meas_.time_stamp = current_z.time_stamp;
      z_last_meas_.z = current_z.z;

      //std::cout << "Difference between current time and accepted measurement: " << (ros::Time::now().toSec() - current_z.time_stamp.toSec()) / dt_pred_ << std::endl;

      // Project state estimate for the remaining time
      int N = (int) (ros::Time::now().toSec() - current_z.time_stamp.toSec());
      for (int i=0; i<N; i++)
         predict();
   }
   else
   {
      if(debug_)
         ROS_WARN("Measurement is too old to be used. Skipping KF update step.");
   }
   


}

void KFTracker::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   Eigen::Vector3d pos;
   pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
   if (pos.norm() > 10000.0)
   {
      ROS_ERROR("[KF poseCallback] Infinite measurement value. Ignoring measurement.");
      return;
   }
      
   z_meas_.time_stamp = msg->header.stamp;
   z_meas_.z(0) = msg->pose.position.x;
   z_meas_.z(1) = msg->pose.position.y;
   z_meas_.z(2) = msg->pose.position.z;

   if(!is_state_initialzed_)
   {
      kf_state_pred_.x = z_meas_.z;
      is_state_initialzed_ = true;
      ROS_INFO("KF state estimate is initialized.");
   }
}

void KFTracker::publishState(void)
{
   geometry_msgs::PoseWithCovarianceStamped msg;
   msg.header.frame_id = tracking_frame_;
   msg.header.stamp = kf_state_pred_.time_stamp;
   
   msg.pose.pose.position.x = kf_state_pred_.x(0);
   msg.pose.pose.position.y = kf_state_pred_.x(1);
   msg.pose.pose.position.z = kf_state_pred_.x(2);
   msg.pose.pose.orientation.w = 1.0; // Idenetity orientation

   auto pxx = kf_state_pred_.P(0,0); auto pyy = kf_state_pred_.P(1,1); auto pzz = kf_state_pred_.P(2,2);
   msg.pose.covariance[0] = pxx; msg.pose.covariance[7] = pyy; msg.pose.covariance[14] = pzz;
   state_pub_.publish(msg);
}

void KFTracker::filterLoop(const ros::TimerEvent& event)
{
   if(is_state_initialzed_)
   {
      // Check if we are still receiving measurements. Otherwise, stop filter as covariance will diverge.
      if( (ros::Time::now().toSec() - z_meas_.time_stamp.toSec()) > measurement_off_time_)
      {
         ROS_ERROR("No measurements received for more than %f seconds. Stopping filter....", measurement_off_time_);
         is_state_initialzed_ = false;
         initKF();
         return;
      }

      if(!predict())
         return;

      if(do_update_step_)
         update();

      publishState();

      // ROS_INFO("Current estimate: x=%f y=%f z=%f", kf_state_pred_.x(0), kf_state_pred_.x(1), kf_state_pred_.x(2));
      // ROS_INFO("Current measurement: x=%f y=%f z=%f", z_meas_.z(0), z_meas_.z(1), z_meas_.z(2));
      
   }
   else
   {
      ROS_WARN_THROTTLE(1, "Filter is not initialized. Requires a measurement first.");
   }
   
}
