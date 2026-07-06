#include "ur5e_curve_path/curve_path_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>
#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ur5e_curve_path::CurvePathNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    });

  int return_code = 0;

  try
  {
    node->initialize();
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "%s",
      error.what());

    return_code = 1;
  }

  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  executor.cancel();

  if (executor_thread.joinable())
  {
    executor_thread.join();
  }

  return return_code;
}
