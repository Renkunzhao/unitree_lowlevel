#pragma once
#include "unitree_lowlevel/gamepad.hpp"
#include "unitree_lowlevel/legged_hal.hpp"

#include <mutex>

#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include "unitree_lowlevel/motor_crc.h"
#include <legged_base/Math.h>   // legged_base::sgn

class Go2Adapter final : public ILeggedAdapter {
  #define TOPIC_LOWCMD "lowcmd"
  #define TOPIC_LOWSTATE "lowstate"
  #define TOPIC_JOYSTICK "wirelesscontroller"
  #define N_JOINTS 12

  static constexpr double PosStopF = (2.146E+9f);
  static constexpr double VelStopF = (16000.0f);

  typedef struct
  {
    uint8_t off; // off 0xA5
    std::array<uint8_t, 3> reserve;
  } BmsCmd;

  typedef struct
  {
    uint8_t mode; // desired working mode
    float q;	  // desired angle (unit: radian)
    float dq;	  // desired velocity (unit: radian/second)
    float tau;	  // desired output torque (unit: N.m)
    float Kp;	  // desired position stiffness (unit: N.m/rad )
    float Kd;	  // desired velocity stiffness (unit: N.m/(rad/s) )
    std::array<uint32_t, 3> reserve;
  } MotorCmd; // motor control

  typedef struct
  {
    std::array<uint8_t, 2> head;
    uint8_t levelFlag;
    uint8_t frameReserve;
      
    std::array<uint32_t, 2> SN;
    std::array<uint32_t, 2> version;
    uint16_t bandWidth;
    std::array<MotorCmd, 20> motorCmd;
    BmsCmd bms;
    std::array<uint8_t, 40> wirelessRemote;
    std::array<uint8_t, 12> led;
    std::array<uint8_t, 2> fan;
    uint8_t gpio;
    uint32_t reserve;
    
    uint32_t crc;
  } LowCmd;           

  void get_crc(unitree_go::msg::LowCmd& msg) {
    LowCmd raw{};
    memcpy(&raw.head[0], &msg.head[0], 2);

    raw.levelFlag = msg.level_flag;
    raw.frameReserve = msg.frame_reserve;

    memcpy(&raw.SN[0], &msg.sn[0], 8);
    memcpy(&raw.version[0], &msg.version[0], 8);

    raw.bandWidth = msg.bandwidth;

    for (int i = 0; i < 20; i++) {
      raw.motorCmd[i].mode = msg.motor_cmd[i].mode;
      raw.motorCmd[i].q = msg.motor_cmd[i].q;
      raw.motorCmd[i].dq = msg.motor_cmd[i].dq;
      raw.motorCmd[i].tau = msg.motor_cmd[i].tau;
      raw.motorCmd[i].Kp = msg.motor_cmd[i].kp;
      raw.motorCmd[i].Kd = msg.motor_cmd[i].kd;

      memcpy(&raw.motorCmd[i].reserve[0], &msg.motor_cmd[i].reserve[0], 12);
    }

    raw.bms.off = msg.bms_cmd.off;
    memcpy(&raw.bms.reserve[0], &msg.bms_cmd.reserve[0], 3);

    memcpy(&raw.wirelessRemote[0], &msg.wireless_remote[0], 40);

    memcpy(&raw.led[0], &msg.led[0], 12);  // go2
    memcpy(&raw.fan[0], &msg.fan[0], 2);
    raw.gpio = msg.gpio;  // go2

    raw.reserve = msg.reserve;

    raw.crc = crc32_core((uint32_t*)&raw, (sizeof(LowCmd) >> 2) - 1);
    msg.crc = raw.crc;
  }

public:
  Go2Adapter() {
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
    updateGamepad(out, joystick_msg_);
    return true;
  }

private:
  static void initLowCmdDefaults(unitree_go::msg::LowCmd& msg) {
    msg.head[0] = 0xFE;
    msg.head[1] = 0xEF;
    msg.level_flag = 0xFF;
    msg.gpio = 0;

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

  void updateGamepad(unitree::common::Gamepad &gamepad, unitree_go::msg::WirelessController &key_msg)
  {
      // update stick values with smooth and deadzone
      gamepad.lx = gamepad.lx * (1 - gamepad.smooth) + (std::fabs(key_msg.lx) < gamepad.dead_zone ? 0.0 : key_msg.lx) * gamepad.smooth;
      gamepad.rx = gamepad.rx * (1 - gamepad.smooth) + (std::fabs(key_msg.rx) < gamepad.dead_zone ? 0.0 : key_msg.rx) * gamepad.smooth;
      gamepad.ry = gamepad.ry * (1 - gamepad.smooth) + (std::fabs(key_msg.ry) < gamepad.dead_zone ? 0.0 : key_msg.ry) * gamepad.smooth;
      gamepad.ly = gamepad.ly * (1 - gamepad.smooth) + (std::fabs(key_msg.ly) < gamepad.dead_zone ? 0.0 : key_msg.ly) * gamepad.smooth;

      // update button states
      unitree::common::xKeySwitchUnion key;
      key.value = key_msg.keys;

      gamepad.R1.update(key.components.R1);
      gamepad.L1.update(key.components.L1);
      gamepad.start.update(key.components.start);
      gamepad.select.update(key.components.select);
      gamepad.R2.update(key.components.R2);
      gamepad.L2.update(key.components.L2);
      gamepad.F1.update(key.components.F1);
      gamepad.F2.update(key.components.F2);
      gamepad.A.update(key.components.A);
      gamepad.B.update(key.components.B);
      gamepad.X.update(key.components.X);
      gamepad.Y.update(key.components.Y);
      gamepad.up.update(key.components.up);
      gamepad.right.update(key.components.right);
      gamepad.down.update(key.components.down);
      gamepad.left.update(key.components.left);
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

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr joystick_sub_;

  unitree_go::msg::LowCmd lowcmd_defaults_;
  unitree_go::msg::LowState lowstate_msg_;
  unitree_go::msg::WirelessController joystick_msg_;

  mutable std::mutex state_mtx_;
  mutable std::mutex joy_mtx_;
  bool has_state_ = false;
  bool has_joy_ = false;
};
