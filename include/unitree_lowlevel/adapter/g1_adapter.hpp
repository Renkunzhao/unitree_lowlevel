
#pragma once
#include "unitree_lowlevel/legged_hal.hpp"

#include <mutex>

#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include "unitree_lowlevel/motor_crc.h"
#include <legged_base/Math.h>   // legged_base::sgn

class G1Adapter final : public ILeggedAdapter {

  #define TOPIC_LOWCMD "lowcmd"
  #define TOPIC_LOWSTATE "lowstate"
  #define N_JOINTS 29

  enum ModePR { PR = 0, AB = 1 };
  enum ModeMachine { Dof23 = 4, Dof29 = 5, Dof27 = 6 };

  typedef struct {
    uint8_t mode;  // desired working mode
    float q;       // desired angle (unit: radian)
    float dq;      // desired velocity (unit: radian/second)
    float tau;     // desired output torque (unit: N.m)
    float Kp;      // desired position stiffness (unit: N.m/rad )
    float Kd;      // desired velocity stiffness (unit: N.m/(rad/s) )
    uint32_t reserve = 0;
  } MotorCmd;  // motor control

  typedef struct {
    uint8_t modePr;
    uint8_t modeMachine;
    std::array<MotorCmd, 35> motorCmd;
    std::array<uint32_t, 4> reserve;
    uint32_t crc;
  } LowCmd;

  void get_crc(unitree_hg::msg::LowCmd &msg) {
    LowCmd raw{};

    raw.modePr = msg.mode_pr;
    raw.modeMachine = msg.mode_machine;

    for (int i = 0; i < 35; i++) {
      raw.motorCmd[i].mode = msg.motor_cmd[i].mode;
      raw.motorCmd[i].q = msg.motor_cmd[i].q;
      raw.motorCmd[i].dq = msg.motor_cmd[i].dq;
      raw.motorCmd[i].tau = msg.motor_cmd[i].tau;
      raw.motorCmd[i].Kp = msg.motor_cmd[i].kp;
      raw.motorCmd[i].Kd = msg.motor_cmd[i].kd;

      raw.motorCmd[i].reserve = msg.motor_cmd[i].reserve;
    }

    memcpy(&raw.reserve[0], &msg.reserve[0], 4);

    raw.crc = crc32_core((uint32_t *)&raw, (sizeof(LowCmd) >> 2) - 1);
    msg.crc = raw.crc;
  }

public:
  G1Adapter() {};

  void setup(rclcpp::Node &node) override {
    lowcmd_pub_ = node.create_publisher<unitree_hg::msg::LowCmd>(
        TOPIC_LOWCMD, 10);

    lowstate_sub_ = node.create_subscription<unitree_hg::msg::LowState>(
        TOPIC_LOWSTATE, 10,
        [this](const unitree_hg::msg::LowState::SharedPtr msg) {
          this->onLowState(msg);
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

    unitree_hg::msg::LowCmd msg;
    msg.mode_pr = ModePR::PR;
    msg.mode_machine = ModeMachine::Dof29;
    for (int i = 0; i < N_JOINTS; ++i) {
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
    std::scoped_lock lk(state_mtx_);
    if (!has_state_) return false;
    unitree::common::REMOTE_DATA_RX rx;
    memcpy(rx.buff, lowstate_msg_.wireless_remote.data(), 40);  // NOLINT
    out.update(rx.RF_RX);
    return true;
  }

private:
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
  }

  void onLowState(const unitree_hg::msg::LowState::SharedPtr msg) {
    std::scoped_lock lk(state_mtx_);
    lowstate_msg_ = *msg;
    has_state_ = true;
    // std::cout << "[G1Adapter] onLowState: Lowstate msg received.\n";
  }

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;

  unitree_hg::msg::LowState lowstate_msg_;

  mutable std::mutex state_mtx_;
  bool has_state_ = false;
};
