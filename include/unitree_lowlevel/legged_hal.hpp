#pragma once
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <mutex>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "unitree_lowlevel/gamepad.hpp"
#include <legged_base/LeggedState.h>
#include <legged_base/Utils.h>
#include <logger/CsvLogger.h>

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

  void log(std::string prefix){
    CsvLogger& logger = CsvLogger::getInstance();

    logger.update(prefix+"q", q);
    logger.update(prefix+"dq", dq);
    logger.update(prefix+"tau", tau);
    logger.update(prefix+"kp", kp);
    logger.update(prefix+"kd", kd);
  }
  
};

class ILeggedAdapter {
public:
  virtual ~ILeggedAdapter() = default;

  virtual void setup(rclcpp::Node &node) = 0;

  virtual void sendJointCmd(const JointCommand &joint_cmd) = 0;
  virtual bool getLeggedState(LeggedState &legged_state) = 0;
  virtual bool getGamePad(unitree::common::Gamepad &gamepad) = 0;

  void logLowStateInputStatistics(
      const rclcpp::Logger &logger,
      std::chrono::steady_clock::time_point now) {
    std::scoped_lock lock(lowstate_stats_mutex_);
    if (!has_lowstate_arrival_ ||
        now - lowstate_stats_window_start_ < std::chrono::seconds(1)) {
      return;
    }

    double hz = 0.0;
    if (lowstate_window_count_ >= 2) {
      const double span_sec = std::chrono::duration<double>(
          lowstate_window_last_ - lowstate_window_first_).count();
      if (span_sec > 0.0) {
        hz = static_cast<double>(lowstate_window_count_ - 1) / span_sec;
      }
    }
    const double max_gap_ms = std::chrono::duration<double, std::milli>(
        lowstate_window_max_gap_).count();
    const double last_age_ms = std::chrono::duration<double, std::milli>(
        now - lowstate_last_arrival_).count();

    RCLCPP_INFO(logger,
                "[lowstate_input] hz=%.1f max_gap_ms=%.2f "
                "last_age_ms=%.2f",
                hz, max_gap_ms, last_age_ms);

    lowstate_stats_window_start_ = now;
    lowstate_window_count_ = 0;
    lowstate_window_max_gap_ = std::chrono::steady_clock::duration::zero();
  }

protected:
  void recordLowStateArrival() {
    const auto now = std::chrono::steady_clock::now();
    std::scoped_lock lock(lowstate_stats_mutex_);

    if (!has_lowstate_arrival_) {
      has_lowstate_arrival_ = true;
      lowstate_stats_window_start_ = now;
    } else {
      lowstate_window_max_gap_ =
          std::max(lowstate_window_max_gap_, now - lowstate_last_arrival_);
    }

    if (lowstate_window_count_ == 0) {
      lowstate_window_first_ = now;
    }
    lowstate_window_last_ = now;
    lowstate_last_arrival_ = now;
    ++lowstate_window_count_;
  }

private:
  std::mutex lowstate_stats_mutex_;
  bool has_lowstate_arrival_ = false;
  size_t lowstate_window_count_ = 0;
  std::chrono::steady_clock::time_point lowstate_stats_window_start_{};
  std::chrono::steady_clock::time_point lowstate_window_first_{};
  std::chrono::steady_clock::time_point lowstate_window_last_{};
  std::chrono::steady_clock::time_point lowstate_last_arrival_{};
  std::chrono::steady_clock::duration lowstate_window_max_gap_{};
};
