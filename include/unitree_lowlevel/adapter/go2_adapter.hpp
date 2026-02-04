#pragma once
#include "unitree_lowlevel/legged_hal.hpp"

#include <iostream>
#include <mutex>
#include <vector>
#include <string>

#include "unitree_lowlevel/motor_crc.h"
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
#include <legged_base/Math.h>   // legged_base::sgn

#define TOPIC_LOWCMD "lowcmd"
#define TOPIC_LOWSTATE "lowstate"
#define TOPIC_JOYSTICK "wirelesscontroller"
#define N_JOINTS 12

class Go2Adapter final : public ILeggedAdapter {
public:
  Go2Adapter() {

    gamepad_.smooth = 0.2;
    gamepad_.dead_zone = 0.5;

    initLowCmdDefaults(lowcmd_defaults_);
  }

  void setup(rclcpp::Node& node) override {

    lowcmd_pub_ = node.create_publisher<unitree_go::msg::LowCmd>(
        TOPIC_LOWCMD, 10);

    lowstate_sub_ = node.create_subscription<unitree_go::msg::LowState>(
        TOPIC_LOWSTATE, 10,
        [this](const unitree_go::msg::LowState::SharedPtr msg) {
          this->onLowState(msg);
        });

    joystick_sub_ = node.create_subscription<unitree_go::msg::WirelessController>(
        TOPIC_JOYSTICK, 10,
        [this](const unitree_go::msg::WirelessController::SharedPtr msg) {
          this->onJoystick(msg);
        });
  }

  void sendJointCmd(const JointCommand& cmd) override {
    if (!lowcmd_pub_) return;

    // 必要维度检查：q/kp/kd 必须匹配 dof
    if (cmd.q.size() != N_JOINTS) return;
    if (cmd.kp.size() != N_JOINTS) return;
    if (cmd.kd.size() != N_JOINTS) return;

    // dq/tau 可选：为空就补 0
    const Eigen::VectorXd dq  = (cmd.dq.size()  == N_JOINTS) ? cmd.dq  : Eigen::VectorXd::Zero(N_JOINTS);
    const Eigen::VectorXd tau = (cmd.tau.size() == N_JOINTS) ? cmd.tau : Eigen::VectorXd::Zero(N_JOINTS);

    unitree_go::msg::LowCmd msg = lowcmd_defaults_;

    for (int i = 0; i < N_JOINTS; ++i) {
      msg.motor_cmd[i].mode = 0x01;  // servo
      msg.motor_cmd[i].q   = cmd.q[i];
      msg.motor_cmd[i].dq  = dq[i];
      msg.motor_cmd[i].kp  = cmd.kp[i];
      msg.motor_cmd[i].kd  = cmd.kd[i];
      msg.motor_cmd[i].tau = tau[i];
    }

    // 如需 CRC：打开下一行
    get_crc(msg);

    lowcmd_pub_->publish(msg);
  }

  bool getLeggedState(LeggedState& out) override {
    std::scoped_lock lk(state_mtx_);
    if (!has_state_) return false;
    updateLeggedState(out);
    return true;
  }

  bool getGamePad(unitree::common::Gamepad& out) override {
    std::scoped_lock lk(joy_mtx_);
    if (!has_joy_) return false;
    gamepad_.Update(joystick_msg_);
    out = gamepad_;
    return true;
  }

private:
  static void initLowCmdDefaults(unitree_go::msg::LowCmd& msg) {
    msg.head[0] = 0xFE;
    msg.head[1] = 0xEF;
    msg.level_flag = 0xFF;
    msg.gpio = 0;

    // 你原 InitLowCmd 是 20 个电机都设默认 stop
    for (int i = 0; i < 20; ++i) {
      msg.motor_cmd[i].mode = 0x01;
      msg.motor_cmd[i].q = PosStopF;
      msg.motor_cmd[i].kp = 0;
      msg.motor_cmd[i].dq = VelStopF;
      msg.motor_cmd[i].kd = 0;
      msg.motor_cmd[i].tau = 0;
    }
  }

  void updateLeggedState(LeggedState& legged_state) {
    // ===== IMU =====
    const double sgn = legged_base::sgn(lowstate_msg_.imu_state.quaternion[0]);
    Eigen::Quaterniond quat_wxyz(
      sgn * lowstate_msg_.imu_state.quaternion[0],
      sgn * lowstate_msg_.imu_state.quaternion[1],
      sgn * lowstate_msg_.imu_state.quaternion[2],
      sgn * lowstate_msg_.imu_state.quaternion[3]);
    legged_state.setBaseRotationFromQuaternion(quat_wxyz);

    legged_state.setBaseAngularVelocityB(Eigen::Vector3d(
      lowstate_msg_.imu_state.gyroscope[0],
      lowstate_msg_.imu_state.gyroscope[1],
      lowstate_msg_.imu_state.gyroscope[2]));

    // ===== joints =====
    Eigen::VectorXd joint_pos(N_JOINTS), joint_vel(N_JOINTS), joint_tau(N_JOINTS);
    for (int i = 0; i < N_JOINTS; ++i) {
      joint_pos[i] = lowstate_msg_.motor_state[i].q;
      joint_vel[i] = lowstate_msg_.motor_state[i].dq;
      joint_tau[i] = lowstate_msg_.motor_state[i].tau_est;
    }
    legged_state.setJointPos(joint_pos);
    legged_state.setJointVel(joint_vel);
    legged_state.setJointTau(joint_tau);

    // ===== foot force -> ee3Dof_fc (z only) =====
    Eigen::VectorXd ee_fc = Eigen::VectorXd::Zero(12);
    for (size_t i = 0; i < 4; ++i) {
      ee_fc[3 * i + 2] = lowstate_msg_.foot_force[i];
    }
    legged_state.setEE3DofFc(ee_fc);
  }

  void onLowState(const unitree_go::msg::LowState::SharedPtr msg) {
    std::scoped_lock lk(state_mtx_);
    lowstate_msg_ = *msg;
    has_state_ = true;
    // std::cout << "[Go2Adapter] onLowState: Lowstate msg received.\n";
  }

  void onJoystick(const unitree_go::msg::WirelessController::SharedPtr msg) {
    std::scoped_lock lk(joy_mtx_);
    joystick_msg_ = *msg;
    has_joy_ = true;
    // std::cout << "[Go2Adapter] onJoystick: Joystick msg received.\n";
  }

private:
  unitree_go::msg::LowCmd lowcmd_defaults_;
  unitree_go::msg::LowState lowstate_msg_;
  unitree_go::msg::WirelessController joystick_msg_;

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr joystick_sub_;

  mutable std::mutex state_mtx_;
  mutable std::mutex joy_mtx_;
  bool has_state_ = false;
  bool has_joy_ = false;

  unitree::common::Gamepad gamepad_;
};
