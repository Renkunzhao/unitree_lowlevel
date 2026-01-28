#include "unitree_lowlevel/motor_crc.h"
#include "unitree_lowlevel/lowlevel_controller.h"
#include <cstddef>
#include <cstdlib>
#include <legged_model/Timer.h>
#include <legged_model/Utils.h>
#include <legged_model/Lie.h>
#include <logger/CsvLogger.h>

#include <cstdint>
#include <string>

#include <yaml-cpp/yaml.h>

// public

LowLevelController::LowLevelController() : rclcpp::Node("low_level_cmd_node") {
  InitLowCmd();

  lowcmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>(TOPIC_LOWCMD, 10);
  lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      TOPIC_LOWSTATE, 10, [this](const unitree_go::msg::LowState::SharedPtr msg) {
        lowcmd_msg_num_++;
        lowstate_msg_ = *msg;
        if (lowcmd_msg_num_ > 10) {
          if (use_sim_timer_ && sim_timer_) {
            sim_timer_->step(lowstate_msg_.tick);
          } else if (!use_sim_timer_) {
            if (!sim_timer_->wall_timer_running()) {
              sim_timer_->start_wall_timer();
            }
          }
        }
      });
  joystick_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
      TOPIC_JOYSTICK, 10, [this](const unitree_go::msg::WirelessController::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joystick_mtx);
        joystick_msg_ = *msg;
        // std::cout << "[LowLevelController] Joystick msg received." << std::endl;
      });

  gamepad_.smooth = 0.2;
  gamepad_.dead_zone = 0.5;
}

void LowLevelController::start(std::string config_file) {
  auto configNode = YAML::LoadFile(config_file);
  std::cout << "[LowLevelController] Load config from " << config_file << std::endl;
  ll_dt_ = configNode["ll_dt"].as<double>();
  use_sim_timer_ = configNode["use_sim_timer"].as<bool>();
  kp_ = LeggedAI::yamlToEigenVector(configNode["PdStand"]["kp"]);
  kd_ = LeggedAI::yamlToEigenVector(configNode["PdStand"]["kd"]);
  qj_lieDown_ = LeggedAI::yamlToEigenVector(configNode["PdStand"]["q_liedown"]);
  qj_stand_ = LeggedAI::yamlToEigenVector(configNode["PdStand"]["q_stand"]);

  x_max_ = configNode["PdStand"]["x_max"].as<double>();
  y_max_ = configNode["PdStand"]["y_max"].as<double>();
  z_min_ = configNode["PdStand"]["z_min"].as<double>();
  z_max_ = configNode["PdStand"]["z_max"].as<double>();
  roll_max_ = configNode["PdStand"]["roll_max"].as<double>();
  pitch_min_ = configNode["PdStand"]["pitch_min"].as<double>();
  pitch_max_ = configNode["PdStand"]["pitch_max"].as<double>();
  yaw_max_ = configNode["PdStand"]["yaw_max"].as<double>();

  // Load LeggedModel
  auto model_config_file = configNode["model_config_file"].as<std::string>();
  std::cout << "[LowLevelController] Load LeggedModel from " << model_config_file << std::endl; 
  robot_model_.loadConfig(YAML::LoadFile(model_config_file));

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
          LowCmdWrite();
        }, LeggedAI::Timer::Mode::SlowSim);
  } else {
    sim_timer_ = std::make_unique<LeggedAI::Timer>(int(ll_dt_ * 1000),  
        [this](int64_t) {
          LowCmdWrite();
        }, LeggedAI::Timer::Mode::Realtime);
  }
}

// private
void LowLevelController::InitLowCmd() {
    lowcmd_msg_.head[0] = 0xFE;
    lowcmd_msg_.head[1] = 0xEF;
    lowcmd_msg_.level_flag = 0xFF;
    lowcmd_msg_.gpio = 0;

    for(int i=0; i<20; i++)
    {
        lowcmd_msg_.motor_cmd[i].mode = (0x01);   // motor switch to servo (PMSM) mode
        lowcmd_msg_.motor_cmd[i].q = (PosStopF);
        lowcmd_msg_.motor_cmd[i].kp = (0);
        lowcmd_msg_.motor_cmd[i].dq = (VelStopF);
        lowcmd_msg_.motor_cmd[i].kd = (0);
        lowcmd_msg_.motor_cmd[i].tau = (0);
    }
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
        lowcmd_msg_.motor_cmd[j].q   = q_ref;
        lowcmd_msg_.motor_cmd[j].dq  = dq_ref;
        lowcmd_msg_.motor_cmd[j].kp  = kp[j];
        lowcmd_msg_.motor_cmd[j].kd  = kd[j];
        lowcmd_msg_.motor_cmd[j].tau = tau_des[j];  // 直接使用外部前馈力矩
    }

    return LeggedAI::smoothstep(t) == 1.0;
}

void LowLevelController::eStop() {
  for (int j = 0; j < 12; j++) {
    lowcmd_msg_.motor_cmd[j].q = 0;
    lowcmd_msg_.motor_cmd[j].dq = 0;
    lowcmd_msg_.motor_cmd[j].kp = 0;
    lowcmd_msg_.motor_cmd[j].kd = 3.0;
    lowcmd_msg_.motor_cmd[j].tau = 0;
  }

  get_crc(lowcmd_msg_);  // Check motor cmd crc
  lowcmd_pub_->publish(lowcmd_msg_);
}

void LowLevelController::torqueClip(){
  for (size_t j=0; j<N_JOINTS; ++j) {
    auto& cmd = lowcmd_msg_.motor_cmd[j];
    auto& state = lowstate_msg_.motor_state[j];
    double tau_eff = cmd.tau + cmd.kp * (cmd.q - state.q) + cmd.kd * (cmd.dq - state.dq);
    const double lim = robot_model_.tauMax()[j];
    const double tau_eff_clamped = std::clamp(tau_eff, -lim, lim);
    cmd.tau += (tau_eff_clamped - tau_eff);
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

void LowLevelController::updateLeggedState() {
  double sgn = Lie::sgn(lowstate_msg_.imu_state.quaternion[0]);
  real_state_.setBaseRotationFromQuaternion(Eigen::Quaternion<double>(
                                    sgn*lowstate_msg_.imu_state.quaternion[0],
                                    sgn*lowstate_msg_.imu_state.quaternion[1],
                                    sgn*lowstate_msg_.imu_state.quaternion[2],
                                    sgn*lowstate_msg_.imu_state.quaternion[3]));

  real_state_.setBaseAngularVelocityB(Eigen::Vector3d(
                                    lowstate_msg_.imu_state.gyroscope[0],
                                    lowstate_msg_.imu_state.gyroscope[1],
                                    lowstate_msg_.imu_state.gyroscope[2]));

  Eigen::VectorXd joint_pos(12), joint_vel(12), joint_tau(12);
  for(int i=0;i<12;i++) {
    joint_pos[i] = lowstate_msg_.motor_state[i].q;
    joint_vel[i] = lowstate_msg_.motor_state[i].dq;
    joint_tau[i] = lowstate_msg_.motor_state[i].tau_est;
  }
  real_state_.setJointPos(joint_pos, robot_model_.jointOrder());
  real_state_.setJointVel(joint_vel, robot_model_.jointOrder());
  real_state_.setJointTau(joint_tau, robot_model_.jointOrder());

  Eigen::VectorXd foot_force = Eigen::VectorXd::Zero(12);
  for(size_t i=0;i<robot_model_.nContacts3Dof();i++) {
    foot_force[3*i+2] = lowstate_msg_.foot_force[i];
  }
  real_state_.setEE3DofFc(foot_force, robot_model_.contact3DofNames());

  updateBaseState();
}

void LowLevelController::LowCmdWrite() {
  auto start = std::chrono::steady_clock::now();

  motiontime_++;

  {
    std::lock_guard<std::mutex> lock(joystick_mtx);
    gamepad_.Update(joystick_msg_);
  }

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
  
  updateLeggedState();

  switch (current_state_) {
    case RobotState::IDLE: {
      for (int j = 0; j < 12; j++) {
        lowcmd_msg_.motor_cmd[j].q = 0;
        lowcmd_msg_.motor_cmd[j].dq = 0;
        lowcmd_msg_.motor_cmd[j].kp = 0;
        lowcmd_msg_.motor_cmd[j].kd = 0;
        lowcmd_msg_.motor_cmd[j].tau = 0;
      }

      // 如果按下START键，进入 STAND
      if (gamepad_.L2.pressed && gamepad_.A.on_press) {
        std::cout << "[LowLevelController] L2 & A: Standing up..." << std::endl;
        switchControllerState();
        current_state_ = RobotState::STAND;
      }
      break;
    }

    case RobotState::STAND: {
      if (interpolateCmd(motiontime_/500., qj_stand_.data(), init_state_.joint_pos().data())) {
        double z_des = z_min_ + (z_max_ - z_min_) * (gamepad_.ly + 1) / 2.0;
        double roll_des = roll_max_ * gamepad_.lx;
        double pitch_des = gamepad_.ry > 0 ? pitch_max_ * gamepad_.ry : pitch_min_ * -gamepad_.ry;
        double yaw_des = yaw_max_ * - gamepad_.rx;

        Eigen::VectorXd jointPos_des(robot_model_.nJoints());
        robot_model_.stanceIK(jointPos_des, {0, 0, z_des}, {yaw_des, pitch_des, roll_des});
        // This line won't work since motiontime has not be setzero
        interpolateCmd(motiontime_/500., jointPos_des.data(), qj_stand_.data());
      }

      // 如果按下STLECT键 -> 躺下 -> 回到IDLE
      if (gamepad_.L2.pressed && gamepad_.A.on_press) {
        std::cout << "[LowLevelController] L2 & A: Lying down..." << std::endl;
        switchControllerState();
        current_state_ = RobotState::LIEDOWN;
      } else if (gamepad_.select.on_press) {
        std::cout << "[LowLevelController] Select: Using High Controller..." << std::endl;
        switchControllerState();
        current_state_ = RobotState::HighController;
        resetHighController();
      }
      break;
    }

    case RobotState::LIEDOWN: {
      if (interpolateCmd(motiontime_/500., qj_lieDown_.data(), init_state_.joint_pos().data())) {
        switchControllerState();
        current_state_ = RobotState::IDLE;
      }
      break;
    }

    case RobotState::HighController: {
      updateHighController();

      if (gamepad_.select.on_press) {
        std::cout << "[LowLevelController] Select: Using Low Controller..." << std::endl;
        switchControllerState();
        current_state_ = RobotState::STAND;
      }
    }
      break;
  }

  torqueClip();
  get_crc(lowcmd_msg_);
  lowcmd_pub_->publish(lowcmd_msg_);

  // 统计
  duration_ms_ = duration<double, std::milli>(steady_clock::now() - start).count();
  loop_cnt_++;
  double real_elapsed = duration<double>(steady_clock::now() - start_wall_).count();
  ave_freq_ = loop_cnt_/real_elapsed;

  log();
}

void LowLevelController::log(){
  CsvLogger& csvLogger = CsvLogger::getInstance();
  csvLogger.update("sim_time", lowstate_msg_.tick*0.001);
  csvLogger.update("controller_time", loop_cnt_*ll_dt_);
  csvLogger.update("state", static_cast<int>(current_state_));
  csvLogger.update("loop_time", duration_ms_);
  csvLogger.update("ave_freq", ave_freq_);
  csvLogger.update("exec_freq", sim_timer_->exec_freq());

  real_state_.log("real_");
  csvLogger.update("com", robot_model_.com(real_state_.custom_state("q_pin")));
  csvLogger.update("vcom", robot_model_.vcom(
                                        real_state_.custom_state("q_pin"),
                                        real_state_.custom_state("v_pin")));
  csvLogger.update("hcom", robot_model_.hcom(
                                        real_state_.custom_state("q_pin"),
                                        real_state_.custom_state("v_pin")));
}
