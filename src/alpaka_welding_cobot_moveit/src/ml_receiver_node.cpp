#include <ml_node_receiver_driver.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MlReceiverMoveitNode>(node_options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
