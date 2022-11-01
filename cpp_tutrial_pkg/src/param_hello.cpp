#include "param_hello.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParamHelloNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}