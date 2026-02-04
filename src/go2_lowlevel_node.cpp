
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include "unitree_lowlevel/lowlevel_controller.h"
#include <logger/CsvLogger.h>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " networkInterface configFile\n";
    rclcpp::shutdown();
    return -1;
  }

  std::cout << "WARNING: Make sure the robot is hung up or lying on the ground.\n"
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  const std::string configFile = argv[2];
  std::cout << "[go2_test] Load config from " << configFile << std::endl;

  YAML::Node configNode = YAML::LoadFile(configFile);

  std::string log_path;
  log_path = legged_base::getEnv("WORKSPACE") + "/" + configNode["log_path"].as<std::string>() + "data.csv";

  CsvLogger& csvLogger = CsvLogger::getInstance();
  csvLogger.setCsvPath(log_path);
  csvLogger.init();

  auto go2_node = std::make_shared<LowLevelController>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(go2_node);

  go2_node->start(configFile);

  exec.spin();

  rclcpp::shutdown();
  csvLogger.save();
  return 0;
}
