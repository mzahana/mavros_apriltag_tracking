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
#ifndef MPC_TRACKER_H
#define MPC_TRACKER_H

#include <exception>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "mavros_apriltag_tracking/KFState.h"
#include "mavros_msgs/PositionTarget.h" // MAVROS setpoint msg
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>

#include <nlopt.hpp>

// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>

class Commander
{
private:
  ros::NodeHandle nh_; /** ROS node handle */
  ros::NodeHandle nh_private_; /** ROS private node handle */
  ros::Subscriber mavstateSub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceServer armAndOffb_service_;
  ros::ServiceServer takeoff_service_;
  ros::ServiceServer holdPos_service_;
  ros::Publisher setpoint_pub_; /** setpoints publisher */

  ros::Timer cmdloop_timer_; /** Setpoint publishing timer */
  double cmd_rate_; /** cmdloop_timer_ rate in seconds */

  mavros_msgs::State current_state_;
  
  ros::Time last_request_; /** Last time of MAV state request (current_state_) */
  ros::Time last_sp_update_req_; /** Last time setpoint_msg_ is updated externally */

  mavros_msgs::PositionTarget setpoint_msg_;
  geometry_msgs::PoseStamped dronePose_msg_; /** Current drone pose */

  double takeoff_alt_;

  bool hold_pos_; /** Flag for holding drone's position */

  /**
   * @brief Commander loop callback.
   */
  void cmdloopCallback(const ros::TimerEvent& event);

  /**
   * @brief ROS service for arming and setting OFFBOARD mode.
   */
  bool armAndOffbCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief ROS service for takeoff
   */
  bool takeoffCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief ROS service for holding current position.
   */
  bool holdPosCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief MAVROS state callback. Contains arming state and flight mode.
   */
  void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);

  /**
   * @brief Sets setpoint mode to position mode and holds current pose.
   */
  void holdPose(void);

public:
  /**
   * @brief Constructor
   * 
   * @param nh ROS node handle
   * @param nh_private Private ROS node handle
   */
  Commander(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  
  /** Destructor */
  ~Commander();

  /**
   * @brief Setter for dronePose_msg_
   */
  void setDronePose(geometry_msgs::PoseStamped msg);

  /**
   * @brief Updates setpoint_msg_ and last_sp_update_req_
   */
  void updateSetpoint(mavros_msgs::PositionTarget msg);

  /**
   * @brief Published setpoint_msg_
   */
  void publishSetpoint(void);
};



//* MPCTracker class
/**
 * Implements Receding horizon controller for tracking a moving object.
 * Assumed model of the moving object is a discrete constant velocity model.
 * Assumed model of the drone tracking the object is a discrete integrator.
 */
class MPCTracker
{
private:
  ros::NodeHandle nh_; /** ROS node handle */
  ros::NodeHandle nh_private_; /** ROS private node handle */
  ros::Subscriber dronePose_sub_; /** Drone's pose subscriber */
  ros::Subscriber targetState_sub_; /** Subscriber to the target Kalman filter state */
  ros::Publisher setpoint_pub_; /** Velocity setpoints publisher */
  ros::ServiceServer engageCtrl_service_; /** Engages/disengages MPC controller */

  const int num_of_states_ = 3; /** Number of states of the vehicle model (integrator in 3D). 3D position = 3.*/
  const int num_of_target_states_ = 6; 
  const int num_of_inputs_ = 3; /** Number of control inputs of the vehicle model. Velocity in 3D = 3. */
  double dt_; /** Prediction time step in seconds */
  Eigen::VectorXd current_drone_state_; /** Current drone state (position and velocity) */
  bool drone_state_received_; /** True if a drone's measurment is received */
  Eigen::VectorXd current_traget_state_; /** Current target state (position and velocity) */
  bool target_state_received_; /** True if a target's measurment is received */
  double alt_above_target_; /** Desired altitude above target. */
  Eigen::VectorXd targetTraj_; /** Target's state trajectory. */
  int mpcWindow; /** Number of prediction steps (N) */
  Eigen::MatrixXd A_; /** Discrete transition matrix drone state space in 3D */
  Eigen::MatrixXd B_; /** Discrete input matrix of a drone state space in 3D */
  double state_weight_; /** State weight. */
  double input_weight_; /** Input weight. */
  Eigen::MatrixXd Q_; /** States wieght matrix of the quadratic MPC objective. x^T Q x. */
  Eigen::MatrixXd R_; /** Inputs wieght matrix of the quadratic MPC objective. u^T R u. */

  Eigen::VectorXd gradient_; /** Gradient vector of the quadratic objective of MPC over a prediciton window. */
  Eigen::MatrixXd hessian_; /** Hessian matrix of the quadratic objective of MPC over a prediciton window. */
  Eigen::SparseMatrix<double> hessian_sparse_; /** sparce version of the hessian */
  Eigen::MatrixXd Ac_; /** Linear constraint  matrix of the QP problem */
  Eigen::SparseMatrix<double> Ac_sparse_; /** Sparse version of Ac */ 
  Eigen::VectorXd xMin_; /** Lower bounds on position and velocity in 3D */
  Eigen::VectorXd xMax_; /** Upper bounds on position and velocity in 3D */
  Eigen::VectorXd uMin_; /** Lower bounds inputs (acceleration) in 3D */
  Eigen::VectorXd uMax_; /** Upper bounds inputs (acceleration) in 3D */
  Eigen::VectorXd lowerBounds_; /** Lower bounds vector of the QP problem */
  Eigen::VectorXd upperBounds_; /** Upper bounds vector of the QP problem */
  double maxVel_; /** Maximum drone's velocity */
  double maxAccel_; /** Maximum drone's acceleration */

  Eigen::Vector3d mpc_ctrl_sol_; /** MPC control solution */
  mavros_msgs::PositionTarget setpoint_msg_;

  bool debug_; /** Enable printing debug messages */
  bool is_MPC_initialized_; /** True if MPC problem is initialized */
  bool engage_controller_; /** Flag for whether to publish MPC controls as setpoints */

  Commander *commander_; /** OFFBOARD commander */

  OsqpEigen::Solver qpSolver_; /** Object of the quadratic program solver */

  /**
   * @brief Predicts target's motion (3D position) over prediction horizon using a constant velocity model.
   * 
   * @param x current target state (position and velocity in 3D)
   * @return Projected target state as Eigen::VectorXd
   */
  Eigen::VectorXd predictTargetState(Eigen::VectorXd x);

  /**
   * @brief Comuptes target's trajectory over prediction horizon of length pred_steps_
   * 
   * @param x_i Initial target state.
   * @return Predicted target trajectory (currently, constant velocity model is supported).
   */
  void computeTargetTraj(Eigen::VectorXd x_i);

  /**
   * @brief Computes control action (acceleration) and corresponding states (position & velocity) uing  MPC controller.
   */
  void computeControlAction(void);

  /**
  * @brief Measurement (3D position and velocity) ROS Callback. Updates current_traget_state_
  * 
  * @param msg Holds KFState state
  * 
  */
  void targetStateCallback(const mavros_apriltag_tracking::KFState::ConstPtr& msg);

  /**
  * @brief Drone's pose ROS Callback. Updates current_vehicle_state_
  * 
  * @param msg Holds PoseStamped measurement
  * 
  */
  void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * @brief Sets the states weight matrix, Q in x^T * Q * x.
   * Uses state_weight_ and updates Q_
   */
  void setQ(void);

  /**
   * @brief Sets the inputs weight matrix, R in u^T * R * u.
   * Uses input_weight_ and updates R_
   */
  void setR(void);

  /**
   * @brief Sets the transition matix for 3D discrete-time  integrator.
   * Result is a function of dt_ and it's saved in A_
   */
  void setTransitionMatrix(void);

  /**
   * @brief Sets the discrete input matrix of a doscrete-time integrator in 3D.
   * Result is a function of dt_ and saved in B_
   */
  void setInputMatrix(void);

  /**
   * @brief Computes the Hessain matrix of the quadratic objective of MPC over a prediciton window, mpcWindow.
   * The Hessian matrix of quadratic function x^T*P*X is P
   * Result is saved in hessian_
   */
  void castMPCToQPHessian(void);

  /**
   * @brief Computes the gradient vector (linear term q in q^T*x) of the quadratic objective of MPC over a prediciton window, mpcWindow.
   * Result is saved in gradient_
   */
  void castMPCToQPGradient(void);

  /**
   * @brief Computes the constraint matrix required by the QP solver
   */
  void castMPCToQPConstraintMatrix(void);

  /**
   * @brief Constructs the bounds vectors for the QP problem.
   * Uses xMin_, xMax_, uMin_, uMax_
   * Result is saved in lowerBound_ and upperBound_ vetors.
   */
  void castMPCToQPConstraintBounds(void);

  /**
   * @brief Initialize QP solver
   * 
   * @return True if initialization is successful. False, otherwise.
   */
  bool initSolver(void);

  /**
   * @brief Initialize MPC problem.
   */
  void initMPC(void);

  /**
   * @brief Executes MPC control loop.
   */
  void mpcLoop(void);

  /**
   * @brief Published setpoint message to MAVROS.
   */
  void publishSetpoint(void);

  void setSetpointFromControl(void);

  bool engageMPCCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

public:

  /**
   * @brief Constructor
   * 
   * @param nh ROS node handle
   * @param nh_private Private ROS node handle
   */
  MPCTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  
  /** Destructor */
  ~MPCTracker();
};

#endif