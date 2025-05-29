#include <rclcpp/rclcpp.hpp>
#include "qbo_arduqbo/drivers/qboduino_driver.h"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  std::string base_port = "/dev/ttyUSB0";  // adapter si besoin
  std::string dummy_port = "";    // aucune tête branchée
  int baud = 115200;

  // Timeout rallongé pour permettre à l’Arduino de booter proprement
  QboDuinoDriver driver(base_port, baud, dummy_port, baud, 4.0, 4.0);


  int board_id = -1, version = -1;

  int code = driver.getVersion("base", board_id, version);
  if (code >= 0 && board_id == 0){
    std::cout << "✅ Base Q.bo Board detected!" << std::endl;
    std::cout << "   ID: " << board_id << ", Version: " << version << std::endl;
  } else {
    std::cerr << "❌ Failed to get base board version" << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
