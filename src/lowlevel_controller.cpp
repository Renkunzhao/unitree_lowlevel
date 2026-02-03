#include "unitree_lowlevel/lowlevel_controller.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "unitree_lowlevel/adapter/go2_adapter.hpp"
#include <legged_base/Timer.h>
#include <legged_base/Utils.h>
#include <legged_base/Math.h>
#include <logger/CsvLogger.h>

// public

LowLevelController::LowLevelController() : rclcpp::Node("low_level_cmd_node") {
}

void LowLevelController::start(std::string config_file) {
  node_ = YAML::LoadFile(config_file);
  std::cout << "[LowLevelController] Load config from " << config_file << std::endl;
  ll_dt_ = node_["ll_dt"].as<double>();
  use_sim_timer_ = node_["use_sim_timer"].as<bool>();
  kp_ = LeggedAI::yamlToEigenVector(node_["kp"]);
  kd_ = LeggedAI::yamlToEigenVector(node_["kd"]);

  // Load LeggedModel
  auto model_config_file = LeggedAI::getEnv("WORKSPACE") + "/" + node_["model_config_file"].as<std::string>();
  std::cout << "[LowLevelController] Load LeggedModel from " << model_config_file << std::endl;
  robot_model_.loadConfig(YAML::LoadFile(model_config_file));

  legged_adapter_ = std::make_unique<Go2Adapter>();
  legged_adapter_->setup(*this);
  jnt_cmd_.resizeZero(robot_model_.nJoints());
  
  // Initialize LeggedState
  des_state_.init(robot_model_.nJoints(), robot_model_.jointOrder(), robot_model_.contact3DofNames(), {});
  real_state_.init(robot_model_.nJoints(), robot_model_.jointOrder(), robot_model_.contact3DofNames(), {});
  robot_model_.creatPinoState(des_state_);
  robot_model_.creatPinoState(real_state_);

  initHighController();

  start_wall_ = std::chrono::steady_clock::now();

  /*loop publishing thread*/
  if (use_sim_timer_) {
    sim_timer_ = std::make_unique<LeggedAI::Timer>(int(ll_dt_ * 1000),  
        [this](int64_t) {
          update();
        }, LeggedAI::Timer::Mode::SlowSim);
  } else {
    sim_timer_ = std::make_unique<LeggedAI::Timer>(int(ll_dt_ * 1000),  
        [this](int64_t) {
          update();
        }, LeggedAI::Timer::Mode::Realtime);
  }
  sim_timer_->start_wall_timer();
}

bool LowLevelController::interpolateCmd(double t,
                                     const double* q_des,
                                     const double* dq_des,
                                     const double* tau_des,
                                     const double* q_init,
                                     const double* dq_init,
                                     const double* kp,
                                     const double* kd) {
    // ===== 插值并生成命令 =====
    for (int j = 0; j < N_JOINTS; ++j)
    {
        // 线性插值位置和速度
        double q_ref  =  LeggedAI::lerp(t, q_init[j], q_des[j]);
        double dq_ref  =  LeggedAI::lerp(t, dq_init[j], dq_des[j]);

        // 写入命令
        jnt_cmd_.q[j]   = q_ref;
        jnt_cmd_.dq[j]  = dq_ref;
        jnt_cmd_.kp[j]  = kp[j];
        jnt_cmd_.kd[j]  = kd[j];
        jnt_cmd_.tau[j] = tau_des[j];  // 直接使用外部前馈力矩
    }

    return LeggedAI::smoothstep(t) == 1.0;
}

void LowLevelController::eStop() {
  for (int j = 0; j < 12; j++) {
    jnt_cmd_.q[j] = 0;
    jnt_cmd_.dq[j]  = 0;
    jnt_cmd_.kp[j]  = 0;
    jnt_cmd_.kd[j]  = 3.0;
    jnt_cmd_.tau[j]  = 0;
  }

  legged_adapter_->sendJointCmd(jnt_cmd_);
}

void LowLevelController::torqueClip(){
  for (size_t j=0; j<N_JOINTS; ++j) {
    double tau_eff = jnt_cmd_.tau[j] + jnt_cmd_.kp[j] * (jnt_cmd_.q[j] - real_state_.joint_pos()[j]) + jnt_cmd_.kd[j] * (jnt_cmd_.dq[j] - real_state_.joint_vel()[j]);
    const double lim = robot_model_.tauMaxOrder()[j];
    const double tau_eff_clamped = std::clamp(tau_eff, -lim, lim);
    jnt_cmd_.tau[j] += (tau_eff_clamped - tau_eff);
    if (tau_eff_clamped!=tau_eff) {
      // std::cout << "[LowLevelController] Leg " << j << " torque out of range [" << tau_eff << "], applying clip." << std::endl;
    }
  }
}

void LowLevelController::switchControllerState() {
  motiontime_ = 0;
  init_state_ = real_state_;
  init_basePos_ = init_state_.base_pos();
  init_baseEulerZYX_ = init_state_.base_eulerZYX();
  init_footPoss_ = robot_model_.contact3DofPoss(init_state_.custom_state("q_pin"));    // note: this is in order of LeggedModel.contact3DofNames
  init_com_ = robot_model_.com(init_state_.custom_state("q_pin"));
  init_qBase.resize(7);
  init_qBase << real_state_.base_pos(), real_state_.base_quat().coeffs();
  std::cout << "[DapcController]" 
            << "\ninit base pos: " << init_state_.base_pos().transpose() 
            << "\ninit com  pos: " << init_com_.transpose()
            << "\ninit footPoss: " 
            << init_footPoss_[0].transpose()
            << init_footPoss_[1].transpose()
            << init_footPoss_[2].transpose()
            << init_footPoss_[3].transpose()
            << std::endl; 
}

void LowLevelController::update() {
  auto start = std::chrono::steady_clock::now();

  motiontime_++;

  legged_adapter_->getGamePad(gamepad_);

  if (gamepad_.L2.pressed && gamepad_.B.on_press) {
    std::cout << "[LowLevelController] Estop start." << std::endl;
    safetyFlag = false;
  }
  if (gamepad_.select.pressed && gamepad_.start.on_press) {
    std::cout << "[LowLevelController] Estop end." << std::endl;
    current_state_ = RobotState::IDLE;
    safetyFlag = true;
  }

  // safefy check
  if (!safetyFlag) {
    eStop();
    return;
  }

  legged_adapter_->getLeggedState(real_state_);
  updateBaseState();

  switch (current_state_) {
  case RobotState::IDLE: {
    for (int j = 0; j < 12; j++) {
      jnt_cmd_.q[j]   = 0;
      jnt_cmd_.dq[j]  = 0;
      jnt_cmd_.kp[j]  = 0;
      jnt_cmd_.kd[j]  = 0;
      jnt_cmd_.tau[j] = 0;
    }

    // L2 + A -> FixStand
    if (gamepad_.L2.pressed && gamepad_.A.on_press) {
      std::cout << "[LowLevelController] L2 & A: Standing up..." << std::endl;
      switchControllerState();
      current_state_ = RobotState::FixStand;
    }
    break;
  }

  case RobotState::FixStand: {
    if (interpolateCmd(motiontime_ / 500.,
                       node_["FixStand"]["q"].as<vector<double>>().data(),
                       init_state_.joint_pos().data()
                      )) {
    }

    // L2 + A -> PreIDLE
    if (gamepad_.L2.pressed && gamepad_.A.on_press) {
      std::cout << "[LowLevelController] L2 & A: Lying down..." << std::endl;
      switchControllerState();
      current_state_ = RobotState::PreIDLE;
      // START -> HIGH CONTROLLER
    } else if (gamepad_.start.on_press) {
      std::cout << "[LowLevelController] Start: Using High Controller..." << std::endl;
      switchControllerState();
      current_state_ = RobotState::HighController;
      resetHighController();
    }
    break;
  }

  case RobotState::PreIDLE: {
    if (interpolateCmd(motiontime_ / 500.,
                       node_["PreIDLE"]["q"].as<vector<double>>().data(),
                       init_state_.joint_pos().data())) {
      switchControllerState();
      current_state_ = RobotState::IDLE;
    }
    break;
  }

  case RobotState::HighController: {
    updateHighController();

    // SELECT -> LOW CONTROLLER
    if (gamepad_.select.on_press) {
      std::cout << "[LowLevelController] Select: Using Low Controller..." << std::endl;
      switchControllerState();
      current_state_ = RobotState::FixStand;
    }
  } 
  break;
  }

  torqueClip();
  legged_adapter_->sendJointCmd(jnt_cmd_);

  // 统计
  duration_ms_ = duration<double, std::milli>(steady_clock::now() - start).count();
  loop_cnt_++;
  double real_elapsed = duration<double>(steady_clock::now() - start_wall_).count();
  ave_freq_ = loop_cnt_/real_elapsed;

  log();
}

void LowLevelController::log(){
  CsvLogger& csvLogger = CsvLogger::getInstance();
  // csvLogger.update("sim_time", lowstate_msg_.tick*0.001);
  csvLogger.update("controller_time", loop_cnt_*ll_dt_);
  csvLogger.update("state", static_cast<int>(current_state_));
  csvLogger.update("loop_time", duration_ms_);
  csvLogger.update("ave_freq", ave_freq_);
  csvLogger.update("exec_freq", sim_timer_->exec_freq());

  real_state_.log("real_");
}
