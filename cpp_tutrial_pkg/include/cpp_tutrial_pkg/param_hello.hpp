#ifndef PARAM_HELLO_HPP
#define PARAM_HELLO_HPP
#include "rclcpp/rclcpp.hpp"

class ParamHelloNode final : public rclcpp::Node
{
public:
  ParamHelloNode();

private:
    std::string param_str_;
};

#endif  // PARAM_HELLO_HPP