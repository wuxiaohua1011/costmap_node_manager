#include "costmap_node_manager/costmap_node_manager.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace costmap_node_manager
{
  CostmapNodeManager::CostmapNodeManager()
      : LifecycleNode("costmap_node_manager", "", true)
  {
    RCLCPP_INFO(get_logger(), "Creating Costmap Manager Node");
    local_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "local_costmap", std::string{get_namespace()}, "local_costmap");
    local_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(local_costmap_ros_);

    global_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "global_costmap", std::string{get_namespace()}, "global_costmap");
    global_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(global_costmap_ros_);
  }

  CostmapNodeManager::~CostmapNodeManager()
  {
    RCLCPP_INFO(get_logger(), "Destroying ROS2Costmap2DNode");
    local_costmap_thread_.reset();
    global_costmap_thread_.reset();
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Configuring");
    local_costmap_ros_->configure();
    global_costmap_ros_->configure();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    local_costmap_ros_->activate();
    global_costmap_ros_->activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    local_costmap_ros_->deactivate();
    global_costmap_ros_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    local_costmap_ros_->cleanup();
    global_costmap_ros_->cleanup();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Shutting Down");
    local_costmap_ros_->shutdown();
    global_costmap_ros_->shutdown();
    return nav2_util::CallbackReturn::SUCCESS;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<costmap_node_manager::CostmapNodeManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}