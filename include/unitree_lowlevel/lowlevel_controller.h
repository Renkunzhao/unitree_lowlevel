#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "unitree_lowlevel/advanced_gamepad.hpp"

#include <legged_base/LeggedState.h>
#include <legged_base/LeggedModel.h>
#include <legged_base/Timer.h>

#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
#include <yaml-cpp/node/node.h>

#define TOPIC_LOWCMD "lowcmd"
#define TOPIC_LOWSTATE "lowstate"
#define TOPIC_JOYSTICK "wirelesscontroller"
#define N_JOINTS 12

class LowLevelController : public rclcpp::Node {
public:
  LowLevelController();
  ~LowLevelController() {
      sim_timer_->report();    
  }

  void start(std::string config_file);

protected:
  void InitLowCmd();
  void LowCmdWrite();

  size_t lowcmd_msg_num_ = 0;
  unitree_go::msg::LowCmd lowcmd_msg_;      // default init
  unitree_go::msg::LowState lowstate_msg_;  // default init
  unitree_go::msg::WirelessController joystick_msg_;
  unitree::common::Gamepad gamepad_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr joystick_sub_;

  std::mutex joystick_mtx;

  double ll_dt_ = 0.002;
  bool use_sim_timer_ = false;
  std::unique_ptr<LeggedAI::Timer> sim_timer_;

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

  static constexpr double ZERO[N_JOINTS] = {0.0f};
  bool interpolateCmd(double t,
                      const double* q_des,
                      const double* dq_des,
                      const double* tau_des,
                      const double* q_init,
                      const double* dq_init,
                      const double* kp,
                      const double* kd);
                      
  bool interpolateCmd(double t,
                      const double* q_des,
                      const double* q_init,
                      const double* kp,
                      const double* kd) {
    return interpolateCmd(t, q_des, ZERO, ZERO, q_init, ZERO, kp, kd);
  }

  bool interpolateCmd(double t,
                      const double* q_des,
                      const double* q_init) {
    return interpolateCmd(t, q_des, ZERO, ZERO, q_init, ZERO, kp_.data(), kd_.data());
  }

private:
  YAML::Node node_;
  
};
