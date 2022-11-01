#include "param_hello.hpp"
#include "rclcpp/rclcpp.hpp"

ParamHelloNode::ParamHelloNode() : Node("hello_param")
{
    RCLCPP_INFO(this->get_logger(), "hello param");
    this->declare_parameter("param_name", "hello_default");
    this->param_str_ = this->get_parameter("param_name").as_string();
    RCLCPP_INFO(this->get_logger(), "%s", this->param_str_.c_str());
};
