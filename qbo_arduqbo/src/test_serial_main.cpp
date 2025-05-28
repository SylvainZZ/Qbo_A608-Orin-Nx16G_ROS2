#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/drivers/qboduino_driver.h"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  std::string base_port = "/dev/ttyUSB0";
  std::string head_port = "/dev/ttyUSB1";
  int board_num, version;

  QboDuinoDriver driver(base_port, 115200, head_port, 115200, 0.01, 0.01);

  if (driver.getVersion("head", board_num, version) == 0) {
    std::cout << "Base board version: " << board_num << "." << version << "\n";
  } else {
    std::cerr << "Error reading base board version\n";
  }

  rclcpp::shutdown();
  return 0;
}
