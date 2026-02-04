#pragma once
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "unitree_lowlevel/advanced_gamepad.hpp"
#include <legged_base/LeggedState.h>
#include <legged_base/Utils.h>

struct JointCommand {
  Eigen::VectorXd q, dq, tau, kp, kd;

  explicit JointCommand(int dof) { resizeZero(dof); }

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

  static JointCommand
  smoothInterp(double t, const Eigen::VectorXd &q_des,
               const Eigen::VectorXd &dq_des, const Eigen::VectorXd &tau_des,
               const Eigen::VectorXd &q_init, const Eigen::VectorXd &dq_init,
               const Eigen::VectorXd &kp, const Eigen::VectorXd &kd) {
    JointCommand result;
    result.resize(q_des.size());
    result.q = legged_base::smoothLerp(t, q_init, q_des);
    result.dq = legged_base::smoothLerp(t, dq_init, dq_des);
    result.kp = kp;
    result.kd = kd;
    result.tau = tau_des;
    return result;
  }

  static JointCommand smoothInterp(double t, const Eigen::VectorXd &q_des,
                                   const Eigen::VectorXd &q_init,
                                   const Eigen::VectorXd &kp,
                                   const Eigen::VectorXd &kd) {
    JointCommand result;
    result.resize(q_des.size());
    result.q = legged_base::smoothLerp(t, q_init, q_des);
    result.dq.setZero();
    result.kp = kp;
    result.kd = kd;
    result.tau.setZero();
    return result;
  }
};

class ILeggedAdapter {
public:
  virtual ~ILeggedAdapter() = default;

  virtual void setup(rclcpp::Node &node) = 0;

  virtual void sendJointCmd(const JointCommand &joint_cmd) = 0;
  virtual bool getLeggedState(LeggedState &legged_state) = 0;
  virtual bool getGamePad(unitree::common::Gamepad &gamepad) = 0;
};
