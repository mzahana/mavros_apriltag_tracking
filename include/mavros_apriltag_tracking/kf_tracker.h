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
#ifndef KF_TRACKER_H
#define KF_TRACKER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "mavros_apriltag_tracking/KFState.h"

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>

//using namespace std;
// using namespace Eigen;

/**
 * Structure to store the current stamped KF prediction
 */
struct kf_state
{
   ros::Time time_stamp;
   Eigen::VectorXd x; // State estimate (from KF prediciton step)
   Eigen::MatrixXd P; // State estimate covariance (from KF prediction step)
};

/**
 * Structure to store current stamped sensor measurement.
 */
struct sensor_measurement
{
   ros::Time time_stamp;
   Eigen::VectorXd z;
};



//* KKFTracker class
/**
 * Implements a Kalman-filter-based object tracker based on constant velocity model
 * Reference 1: Constant velocity model (http://www.robots.ox.ac.uk/~ian/Teaching/Estimation/LectureNotes2.pdf)
 * Reference 2: Discrete KF equations (https://en.wikipedia.org/wiki/Kalman_filter)
 */
class KFTracker
{
private:
   ros::NodeHandle nh_;
   ros::NodeHandle nh_private_;

   kf_state kf_state_pred_; /**< KF predicted state and covariance */
   Eigen::VectorXd x_pred_; /**< State estimate. 6x1 vector. 3D position and velocity. */
   Eigen::MatrixXd P_pred_; /**< Updated (a posteriori) estimate covariance. */
   sensor_measurement z_meas_; /**< current sensor measurement. 3x1 vector. */
   sensor_measurement z_last_meas_; /**< previous sensor measurement. 3x1 vector. */
   Eigen::MatrixXd F_; /**< State transition model. */
   Eigen::MatrixXd H_; /**< Observation model. */
   Eigen::MatrixXd Q_; /**< Covariance of the process noise. */
   Eigen::MatrixXd R_; /**< Covariance of the observation noise. */
   double q_; /**< standard deviation of process noise. */
   double r_; /**< standard deviation of observation noise. */
   double dt_pred_; /**< KF prediction sampling time in seconds. */
   bool is_state_initialzed_; /**< flag to start state prediction. Initialized by 1st measurement. */
   std::vector<kf_state> state_buffer_; /**< Bueffer to store last state_buffer_size_ predicted x and P */
   unsigned int state_buffer_size_; /**< lenght of state buffer state_buffer_*/
   std::string tracking_frame_; /**< Coordinate frame at which tracking is performed */
   bool do_update_step_; /**< Whether to perform the KF update step. WARNING this is just for debuggin. */
   ros::Time last_measurement_time_; /**<  Time of the last received measurement*/
   double measurement_off_time_; /**< Maximum time (in seconds) with no measurement before filter is stopped. */

   bool debug_; /**< for printing debug message */

   bool use_constant_accel_model_; /**< Using transition and process noise cov matrices for constant acceleration model */
   bool use_pos_and_vel_observations_; /**< Set true if observing position and velocity. */

   /**
    * @brief Computes estimate covariance Q based on sampling time dt_pred_ and  process noise q_.
    */
   void setQ(void);

   /**
    * @brief Computes observation covariance R based on  observation noise r_.
    */
   void setR(void);

   /**
    * @brief Computes discrete transition matrix F_ based on sampling time dt_pred_.
    */
   void setF(void);

   /**
    * @brief Computes discrete observation matrix H_. Observations are positions x,y,z.
    */
   void setH(void);

   /**
    * @brief Initialize estimate covariance P with Q.
    */
   void initP(void);

   /**
    * @brief Initializes KF F,H,Q,R, and initial state and covariance estimates
    */
   void initKF(void);

   /**
    * @brief Adds the current x_pred_ and P_pred_state to the end of buffer state_buffer_  .
    * It erases the first element in the state_buffer_ vector if its size exceeds state_buffer_size_.
    */
   void updateStateBuffer(void);

   /**
    * @brief Performs Kalman filter prediction step. Result is stored in kf_state_pred_.
    * @return true if prediciton is successful (e.g. state does not explode!)
    */
   bool predict(void);

   /**
    * @brief Performs Kalman filter update step.
    */
   void update(void);

   /**
    * @brief Executes the KF predict and update steps.
    */
   void filterLoop(const ros::TimerEvent& event);

   /**
    * @brief Measurement ROS Callback. Updates z_meas_
    * 
    * @param msg Holds PoseStamped measurement
    */
   void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

   /**
    * @brief Publishes state estimate (kf_state_pred_.x) in a ROS PoseStamped topic.
    * Orientation in PoseStampe is ignored and set to Identity
    */
   void publishState(void);

   ros::Publisher posState_pub_; /** Publisher for position part only */
   ros::Publisher kfState_pub_; /** Publisher for the entire KF state */
   ros::Subscriber pose_sub_; /**< Subscriber to measurments. */
   ros::Timer kf_loop_timer_;

public:
   KFTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
   ~KFTracker();
};

#endif