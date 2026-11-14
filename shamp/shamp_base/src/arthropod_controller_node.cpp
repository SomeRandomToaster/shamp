#include "arthropod_base/arthropod_controller.h"

int main(int argc, char** argv )
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArthropodController>());
  rclcpp::shutdown();
  return 0;
}