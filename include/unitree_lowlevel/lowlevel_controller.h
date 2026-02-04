#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Dense>
#include <yaml-cpp/node/node.h>
#include <rclcpp/rclcpp.hpp>

#include "unitree_lowlevel/legged_hal.hpp"

#include <legged_base/LeggedState.h>
#include <legged_base/LeggedModel.h>
#include <legged_base/Timer.h>

#define N_JOINTS 12

class LowLevelController : public rclcpp::Node {
public:
  LowLevelController();
  ~LowLevelController() {
      sim_timer_->report();    
  }

  void start(std::string config_file);

protected:
  void update();

  JointCommand jnt_cmd_;
  unitree::common::Gamepad gamepad_;
  
  std::unique_ptr<ILeggedAdapter> legged_adapter_;

  double ll_dt_ = 0.002;
  bool use_sim_timer_ = false;
  std::unique_ptr<legged_base::Timer> sim_timer_;

  // statistic info for logging and debugging
  std::chrono::steady_clock::time_point start_wall_;
  size_t loop_cnt_ = 0;
  double duration_ms_ = 0, ave_freq_ = 0;
  size_t motiontime_ = 0;

  void switchControllerState();
  void updateLeggedState();
  virtual void updateBaseState() {};
  virtual void initHighController() {};
  virtual void resetHighController() {};
  virtual void updateHighController() {};
  virtual void log();

  bool safetyFlag = true;
  void eStop();
  void torqueClip();

  LeggedState real_state_, des_state_, init_state_;
  LeggedModel robot_model_;
  Eigen::Vector3d init_com_, init_basePos_, init_baseEulerZYX_;
  VectorXd init_qBase;
  std::vector<Eigen::Vector3d> init_footPoss_;

  VectorXd tau_max_, kp_, kd_;

  enum class RobotState : int {
      IDLE       = 0,
      FixStand      = 1,
      PreIDLE    = 2,
      HighController = 3
  };

  RobotState current_state_ = RobotState::IDLE;

private:
  YAML::Node node_;
  
};
