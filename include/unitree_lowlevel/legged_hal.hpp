#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include "unitree_lowlevel/advanced_gamepad.hpp"
#include <legged_base/LeggedState.h>

struct JointCommand {
  Eigen::VectorXd q, dq, tau, kp, kd;

  explicit JointCommand(int dof) {
    resizeZero(dof);
  }

  JointCommand() = default;

  void resize(int dof) {
    q.resize(dof);
    dq.resize(dof);
    tau.resize(dof);
    kp.resize(dof);
    kd.resize(dof);
  }

  void resizeZero(int dof) {
    resize(dof);
    q.setZero();
    dq.setZero();
    tau.setZero();
    kp.setZero();
    kd.setZero();
  }
};


class ILeggedAdapter {
public:
  virtual ~ILeggedAdapter() = default;

  virtual void setup(rclcpp::Node& node) = 0;

  virtual void sendJointCmd(const JointCommand& joint_cmd) = 0;
  virtual bool getLeggedState(LeggedState& legged_state) = 0;
  virtual bool getGamePad(unitree::common::Gamepad& gamepad) = 0;
};
