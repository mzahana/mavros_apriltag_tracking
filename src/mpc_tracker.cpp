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
#include "mavros_apriltag_tracking/mpc_tracker.h"

MPCTracker::MPCTracker(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
nh_(nh),
nh_private_(nh_private),
dt_(0.05),
mpcWindow(20),
state_weight_(1.0),
input_weight_(0.1),
maxVel_(5.0),
maxAccel_(2.5),
alt_above_target_(1.0),
is_MPC_initialized_(false),
drone_state_received_(false),
target_state_received_(false),
debug_(true),
engage_controller_(false)
{
   nh_private.param("debug", debug_, false);
   nh_private_.param("mpc_window", mpcWindow, 20);
   nh_private_.param("state_weight", state_weight_, 1.0);
   nh_private_.param("input_weight", input_weight_, 0.1);
   nh_private_.param("maxVel", maxVel_, 5.0);
   nh_private_.param("maxAccel", maxAccel_, 2.5);
   nh_private_.param("alt_above_target", alt_above_target_, 2.0);
   
   dronePose_sub_ =  nh_.subscribe("mavros/local_position/pose", 1, &MPCTracker::dronePoseCallback, this);
   targetState_sub_ = nh_.subscribe("kf/state", 1, &MPCTracker::targetStateCallback, this);

   engageCtrl_service_ = nh_.advertiseService("mpc_commander/engage", &MPCTracker::engageMPCCallback, this);

   commander_ = new Commander(nh, nh_private);

   initMPC();

  //nlopt::opt *opt = new nlopt::opt(nlopt::LD_MMA, num_of_states_*mpcWindow + num_of_inputs_*mpcWindow);

}

MPCTracker::~MPCTracker()
{
   /* Destructor */
}

bool MPCTracker::engageMPCCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
   engage_controller_ = req.data;
   if(engage_controller_)
      res.message = "MPC controller is engaged";
   else
      res.message = "MPC controller is disengaged";

   res.success = 0;
   
   return true;
}

void MPCTracker::dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   commander_->setDronePose(*msg);
   current_drone_state_.resize(num_of_states_);
   current_drone_state_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
   drone_state_received_ = true;
}

void MPCTracker::targetStateCallback(const mavros_apriltag_tracking::KFState::ConstPtr& msg)
{
   current_traget_state_.resize(num_of_target_states_); // 3D position and velocity
   current_traget_state_ << msg->position.x, msg->position.y, msg->position.z,
                           msg->velocity.x, msg->velocity.y, msg->velocity.z;

   target_state_received_ = true;

   if(drone_state_received_)
   {
      mpcLoop();
      setSetpointFromControl();
      if(engage_controller_)
         commander_->updateSetpoint(setpoint_msg_);
   }
}

Eigen::VectorXd MPCTracker::predictTargetState(Eigen::VectorXd x)
{
   Eigen::MatrixXd F(6,6);
   F = Eigen::MatrixXd::Identity(6,6);
   F(0,3) = dt_; // x - vx
   F(1,4) = dt_; // y - vy
   F(2,5) = dt_; // z - vz

   return F*x;
}

void MPCTracker::computeTargetTraj(Eigen::VectorXd x_i)
{
   Eigen::VectorXd x = x_i;

   //Eigen::VectorXd traj(num_of_states_*mpcWindow,1);
   targetTraj_.resize(num_of_states_*mpcWindow);

   for(unsigned int i=0; i<mpcWindow; i++)
   {
      x = predictTargetState(x);
      auto x_des = x;
      x_des(2) = x(2) + alt_above_target_; // desired altitude above target
      targetTraj_.block(i*num_of_states_,0,num_of_states_,1) = x_des.segment(0,num_of_states_);
   }

   if(debug_)
   {
      std::cout << "Target trajectory: " << std::endl<< targetTraj_ <<std::endl;
   }

}

void MPCTracker::setTransitionMatrix(void)
{
   A_.resize(num_of_states_,num_of_states_);
   A_ = Eigen::MatrixXd::Identity(num_of_states_,num_of_states_); // Discrete integrator model
   // A_(0,3) = dt_; // x - vx
   // A_(1,4) = dt_; // y - vy
   // A_(2,5) = dt_; // z - vz

   if(debug_)
   {
      std::cout<< "Transition matrix: " <<std::endl<<A_<<std::endl;
   }
}

void MPCTracker::setInputMatrix(void)
{
   B_.resize(num_of_states_, num_of_inputs_);
   B_ = dt_* Eigen::MatrixXd::Identity(num_of_states_, num_of_inputs_); // Discrete integrator in 3D
   // Eigen::MatrixXd upper = dt_*dt_* Eigen::MatrixXd::Identity(3,3);
   // Eigen::MatrixXd lower = dt_* Eigen::MatrixXd::Identity(3,3);
   // // (start_row, start_col, num_rows, num_cols)
   // B_.block(0,0,num_of_inputs_, num_of_inputs_) = upper;
   // B_.block(3,0,num_of_inputs_, num_of_inputs_) = lower;

   if(debug_)
   {
      std::cout<<"Input matrix: " << std::endl<<B_<<std::endl;
   }
}

void MPCTracker::setQ(void)
{
   Q_.resize(num_of_states_,num_of_states_);
   Q_ = state_weight_* Eigen::MatrixXd::Identity(num_of_states_, num_of_states_);

   if(debug_)
   {
      std::cout<<"Q matrix: "<<std::endl<<Q_<<std::endl;
   }
}

void MPCTracker::setR(void)
{
   R_.resize(num_of_inputs_,num_of_inputs_);
   R_ = input_weight_* Eigen::MatrixXd::Identity(num_of_inputs_, num_of_inputs_);

   if(debug_)
   {
      std::cout<<"R matrix: "<<std::endl<<R_<<std::endl;
   }
}

void MPCTracker::castMPCToQPHessian(void)
{
   int h_size = (num_of_inputs_+num_of_states_)*mpcWindow;
   hessian_.resize(h_size, h_size);
   hessian_ = Eigen::MatrixXd::Zero(h_size, h_size);

   // Set Q
   for (unsigned int i=0; i<mpcWindow; i++)
   {
      //hessian_.block<num_of_states_,num_of_states_>(num_of_states_*i, num_of_states_*i) = Q_;
      hessian_.block(num_of_states_*i, num_of_states_*i, num_of_states_, num_of_states_) = Q_;
   }

   // Set R
   unsigned int idx = mpcWindow*num_of_states_; //initial index after setting Q
   for (unsigned int i=0; i<mpcWindow; i++)
   {
      //hessian_.block<num_of_inputs_, num_of_inputs_>(idx+i*num_of_inputs_,idx+i*num_of_inputs_) = R_;
      hessian_.block(idx+i*num_of_inputs_, idx+i*num_of_inputs_, num_of_inputs_, num_of_inputs_) = R_;
   }

   if(debug_)
   {
      std::cout<<"QP Hessian matrix: "<<std::endl<<hessian_<<std::endl;
   }
}

void MPCTracker::castMPCToQPGradient(void)
{
   int g_size = (num_of_inputs_+num_of_states_)*mpcWindow;
   gradient_.resize(g_size);
   gradient_.setZero();
   for(unsigned int i=0; i<mpcWindow; i++)
   {
      //gradient_.block(i*num_of_states_,0,num_of_states_,1) = Q_*targetTraj_.block(i*num_of_states_,0,num_of_states_,1);
      gradient_.segment(i*num_of_states_,num_of_states_) = -2.0*Q_*targetTraj_.segment(i*num_of_states_,num_of_states_);
   }

   if(debug_)
   {
      std::cout<<"QP gradient vector: "<<std::endl<<gradient_<<std::endl;
   }
}


void MPCTracker::castMPCToQPConstraintMatrix(void)
{
   int size_r = (2*num_of_states_ + num_of_inputs_)*mpcWindow;
   int size_c = (num_of_states_ + num_of_inputs_)*mpcWindow;
   Ac_.resize(size_r, size_c);

   // State block = Upper left block
   Eigen::MatrixXd state_block = Eigen::MatrixXd::Identity(num_of_states_*mpcWindow, num_of_states_*mpcWindow);
   for (unsigned int i=1; i<mpcWindow; i++)
   {
      state_block.block(i*num_of_states_, (i-1)*num_of_states_, num_of_states_, num_of_states_) = -1.0*A_;
   }

   // input block = Upper right block
   Eigen::MatrixXd input_block = Eigen::MatrixXd::Zero(num_of_states_*mpcWindow, num_of_inputs_*mpcWindow);
   for(unsigned int i=0; i<mpcWindow; i++)
   {
      input_block.block(i*num_of_states_, i*num_of_inputs_, num_of_states_,num_of_inputs_) = -1.0*B_;
   }

   // State bounds block = middle block
   Eigen::MatrixXd state_bound_block = Eigen::MatrixXd::Zero(num_of_states_*mpcWindow, size_c);
   for(unsigned int i=0; i<mpcWindow; i++)
   {
      state_bound_block.block(i*num_of_states_, i*num_of_states_, num_of_states_, num_of_states_).setIdentity();
   }

   // Input bounds block = Lower block
   Eigen::MatrixXd input_bound_block = Eigen::MatrixXd::Zero(num_of_inputs_*mpcWindow, size_c);
   unsigned int idx = mpcWindow*num_of_states_;
   for(unsigned int i=0; i<mpcWindow; i++)
   {
      input_bound_block.block(i*num_of_inputs_, idx+i*num_of_inputs_, num_of_inputs_, num_of_inputs_).setIdentity();
   }

   // Finally evaluate the constraint matrix !
   Ac_.block(0, 0, num_of_states_*mpcWindow, num_of_states_*mpcWindow) = state_block;
   Ac_.block(0, num_of_states_*mpcWindow, num_of_states_*mpcWindow, num_of_inputs_*mpcWindow) = input_block;
   Ac_.block(num_of_states_*mpcWindow, 0, num_of_states_*mpcWindow, size_c) = state_bound_block;
   Ac_.block(2*num_of_states_*mpcWindow, 0, num_of_inputs_*mpcWindow, size_c) = input_bound_block;

   if(debug_)
   {
      std::cout<<"Constraint matrix: " <<std::endl<<Ac_<<std::endl;
   }
}

void MPCTracker::castMPCToQPConstraintBounds(void)
{
   /** I am so hungry! I Need to EAT! */
   lowerBounds_ = Eigen::VectorXd::Zero((2*num_of_states_+num_of_inputs_)*mpcWindow);
   upperBounds_ = Eigen::VectorXd::Zero((2*num_of_states_+num_of_inputs_)*mpcWindow);
   lowerBounds_.segment(0,num_of_states_) = A_*current_drone_state_;
   upperBounds_.segment(0,num_of_states_) = A_*current_drone_state_;

   Eigen::VectorXd xMin(num_of_states_), xMax(num_of_states_); // px ,py, pz, vx, vy, vz
   xMin << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;//, -maxVel_, -maxVel_, -maxVel_;
   xMax << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;//, maxVel_, maxVel_, maxVel_;

   Eigen::Vector3d uMin(num_of_inputs_), uMax(num_of_inputs_); // accel_x, accel_y, accel_z
   // uMin << -maxAccel_, -maxAccel_, -maxAccel_;
   // uMax << maxAccel_, maxAccel_, maxAccel_;
   uMin << -maxVel_, -maxVel_, -maxVel_;
   uMax << maxVel_, maxVel_, maxVel_;

   unsigned int idx = num_of_states_*mpcWindow;
   for(unsigned int i=0; i<mpcWindow; i++)
   {
      lowerBounds_.segment(idx+i*num_of_states_, num_of_states_) = xMin;
      upperBounds_.segment(idx+i*num_of_states_, num_of_states_) = xMax;
   }

   idx = 2*num_of_states_*mpcWindow;
   for(unsigned int i=0; i<mpcWindow; i++)
   {
      lowerBounds_.segment(idx+i*num_of_inputs_, num_of_inputs_) = uMin;
      upperBounds_.segment(idx+i*num_of_inputs_, num_of_inputs_) = uMax;
   }

   if(debug_)
   {
      std::cout<<"Lower bounds vector: "<<std::endl<<lowerBounds_<<std::endl;
      std::cout<<"Upper bounds vector: "<<std::endl<<upperBounds_<<std::endl;
   }
}

bool MPCTracker::initSolver(void)
{
   ROS_INFO("Initializing solver ...");
   qpSolver_.settings()->setVerbosity(false);
   qpSolver_.settings()->setWarmStart(true);
   // set the initial data of the QP solver
   qpSolver_.data()->setNumberOfVariables((num_of_states_+num_of_inputs_)*mpcWindow);
   qpSolver_.data()->setNumberOfConstraints((2*num_of_states_+num_of_inputs_)*mpcWindow);
   hessian_sparse_ = hessian_.sparseView();
   Ac_sparse_ = Ac_.sparseView();
   if(!qpSolver_.data()->setHessianMatrix(hessian_sparse_)) return false;
   if(!qpSolver_.data()->setGradient(gradient_)) return false;
   if(!qpSolver_.data()->setLinearConstraintsMatrix(Ac_sparse_)) return false;
   if(!qpSolver_.data()->setLowerBound(lowerBounds_)) return false;
   if(!qpSolver_.data()->setUpperBound(upperBounds_)) return false;

   if(!qpSolver_.initSolver()) return false;


   if(debug_)
   {
      ROS_INFO("QP solver is initialized.");
   }

}

void MPCTracker::initMPC(void)
{
   ros::WallTime startTime = ros::WallTime::now();
   setTransitionMatrix(); 
   setInputMatrix();
   setQ();
   setR();
   castMPCToQPHessian();
   current_drone_state_.resize(num_of_states_);
   current_drone_state_ = Eigen::VectorXd::Random(num_of_states_);
   current_traget_state_.resize(num_of_target_states_);
   current_traget_state_ = Eigen::VectorXd::Random(num_of_target_states_);
   computeTargetTraj(current_traget_state_);
   castMPCToQPGradient();
   castMPCToQPConstraintMatrix();
   castMPCToQPConstraintBounds();
   initSolver();

   is_MPC_initialized_ = true;

   if(debug_)
   {
      double total_elapsed = (ros::WallTime::now() - startTime).toSec();
      ROS_INFO("MPC is initialized in %f second(s).", total_elapsed);
   }

   // drone_state_received_ = true; // This is for debugging. SHOULD BE REMOVED
   // target_state_received_ = true; // This is for debugging. SHOULD BE REMOVED
   // mpcLoop();// This is for debugging. SHOULD BE REMOVED
}

void MPCTracker::mpcLoop(void)
{
   ros::WallTime startTime = ros::WallTime::now();

   if(!is_MPC_initialized_)
   {
      ROS_WARN("MPC controller is not initialized. Skiipin MPC loop.");
      return;
   }
   if(!drone_state_received_)
   {
      ROS_WARN("[MPC controller] Drone state is not received yet. Skipping MPC loop.");
      return;
   }

   // update current drone's position (updates QP linear constraints bounds)
   //std::cout << "Current drone state" << std::endl << current_drone_state_ << std::endl;
   //std::cout << "Current target state" << std::endl << current_traget_state_ << std::endl;
   lowerBounds_.segment(0,num_of_states_) = A_*current_drone_state_;
   upperBounds_.segment(0,num_of_states_) = A_*current_drone_state_;
   if(!qpSolver_.updateBounds(lowerBounds_, upperBounds_)) return;
   if(debug_)
      ROS_INFO("Bounds are updated.");

   //update target trajectory, then update QP gradient vector
   computeTargetTraj(current_traget_state_);
   std::cout << "Target trajectory: " << std::endl<< targetTraj_<< std::endl;
   if(debug_)
      ROS_INFO("Target trajectory  is updated.");
   castMPCToQPGradient();
   if(!qpSolver_.updateGradient(gradient_)) return;
   if(debug_)
      ROS_INFO("QP gradient is updated");

   // Solve MPC
   if(!qpSolver_.solve())
   {
      ROS_WARN("MPC solution is not found");
      return;
   }

   // Extract control input
   Eigen::VectorXd QPSolution;
   QPSolution = qpSolver_.getSolution();
   mpc_ctrl_sol_ = QPSolution.block(num_of_states_ * mpcWindow, 0, num_of_inputs_, 1); // 3D velocity
   std::cout << "optimal trajectory: " << std::endl << QPSolution.block(0,0, num_of_states_*mpcWindow,1) << std::endl;

   if(true)
   {
      std::cout << "Control solution: " << std::endl << mpc_ctrl_sol_ << std::endl;
   }

   if(debug_)
   {
      double total_elapsed = (ros::WallTime::now() - startTime).toSec();
      ROS_INFO("MPC is solved in %f second(s).", total_elapsed);
   }
}

void MPCTracker::setSetpointFromControl(void)
{
   setpoint_msg_.header.stamp = ros::Time::now();
   setpoint_msg_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
   setpoint_msg_.type_mask = mavros_msgs::PositionTarget::IGNORE_PX + mavros_msgs::PositionTarget::IGNORE_PY + mavros_msgs::PositionTarget::IGNORE_PZ +
                           mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + 
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
   setpoint_msg_.velocity.x = mpc_ctrl_sol_(0);
   setpoint_msg_.velocity.y = mpc_ctrl_sol_(1);
   setpoint_msg_.velocity.z = mpc_ctrl_sol_(2);
   setpoint_msg_.yaw = 0.0; // TODO: ??
}


/** ******************************** Commander class ******************************** */


Commander::Commander(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
nh_(nh),
nh_private_(nh_private),
takeoff_alt_(2.0),
hold_pos_(false)
{
   nh_private.param("takeoff_alt", takeoff_alt_, 2.0);
   nh_private.param("commander_rate", cmd_rate_, 0.05);

   mavstateSub_ = nh_.subscribe("mavros/state", 1, &Commander::mavstateCallback, this,ros::TransportHints().tcpNoDelay());

   setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

   arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
   set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
   armAndOffb_service_ =  nh_.advertiseService("mpc_commander/arm_and_offboard", &Commander::armAndOffbCallback, this);
   takeoff_service_ = nh_.advertiseService("mpc_commander/takeoff", &Commander::takeoffCallback, this);
   holdPos_service_ = nh_.advertiseService("mpc_commander/hold", &Commander::holdPosCallback, this);
   
   cmdloop_timer_ = nh_.createTimer(ros::Duration(cmd_rate_), &Commander::cmdloopCallback, this); // Define timer for constant loop rate
}

Commander::~Commander(){ /** Desturctor */ }

bool Commander::armAndOffbCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
   mavros_msgs::CommandBool arm_cmd;
   mavros_msgs::SetMode offb_set_mode;
   arm_cmd.request.value = true;
   offb_set_mode.request.custom_mode = "OFFBOARD";

   if( arming_client_.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
   }
   if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent){
      ROS_INFO("Offboard enabled");
   }
   return true;
}

bool Commander::holdPosCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
   holdPose();
}

bool Commander::takeoffCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
   last_sp_update_req_ = ros::Time::now();

   setpoint_msg_.header.stamp = ros::Time::now();
   setpoint_msg_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
   setpoint_msg_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + 
                              mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY + mavros_msgs::PositionTarget::IGNORE_VZ + 
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
   setpoint_msg_.yaw = 0.0; // TODO: ??
   setpoint_msg_.position.x = dronePose_msg_.pose.position.x;
   setpoint_msg_.position.y = dronePose_msg_.pose.position.y;
   setpoint_msg_.position.z = dronePose_msg_.pose.position.z + takeoff_alt_;

   mavros_msgs::CommandBool arm_cmd;
   mavros_msgs::SetMode offb_set_mode;
   arm_cmd.request.value = true;
   offb_set_mode.request.custom_mode = "OFFBOARD";

   if( arming_client_.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
   }
   if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent){
      ROS_INFO("Offboard enabled");
   }

   ROS_INFO("Position setpoint is updated for takeoff");
   
   return true;
}

void Commander::mavstateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void Commander::setDronePose(geometry_msgs::PoseStamped msg)
{
   dronePose_msg_ = msg;
}

void Commander::updateSetpoint(mavros_msgs::PositionTarget msg)
{
   last_sp_update_req_ = ros::Time::now();
   setpoint_msg_ = msg;
}

void Commander::holdPose(void)
{
   setpoint_msg_.header.stamp = ros::Time::now();
   setpoint_msg_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
   setpoint_msg_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + 
                              mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY + mavros_msgs::PositionTarget::IGNORE_VZ + 
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
   setpoint_msg_.yaw = 0.0; // TODO: ??
   setpoint_msg_.position.x = dronePose_msg_.pose.position.x;
   setpoint_msg_.position.y = dronePose_msg_.pose.position.y;
   setpoint_msg_.position.z = dronePose_msg_.pose.position.z;
}

void Commander::publishSetpoint(void)
{
   // if( (ros::Time::now() - last_sp_update_req_) > ros::Duration(2.0))
   // {
   //    ROS_WARN_THROTTLE(1, "Setpoint is not updated externally. Holding at current position.");
   //    if(!hold_pos_)
   //    {
   //       holdPose();
   //       hold_pos_ = true;
   //    }
   // }
   // else
   // {
   //    hold_pos_ = false;
   // }
   
   setpoint_pub_.publish(setpoint_msg_);
}

void Commander::cmdloopCallback(const ros::TimerEvent& event)
{
   publishSetpoint();
}