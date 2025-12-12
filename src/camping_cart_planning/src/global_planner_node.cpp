/**
 * HH_251203: Global Planner Node
 * 
 * Refactored from camping_cart_navigation to camping_cart_planning.
 * Subscribes to RViz 2D Nav Goal and publishes planned global path.
 * TODO: Integrate with camping_cart_map for actual routing.
 * 
 * See: /home/hong/camping_cart_ws/src/camping_cart_planning/README.md
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <memory>

class GlobalPlannerNode : public rclcpp::Node {
public:
  GlobalPlannerNode() : Node("global_planner_node") {
    // Create subscribers and publishers
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/global_path", 10);

    RCLCPP_INFO(this->get_logger(), "Global Planner Node initialized");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), 
      "Goal received: (" << msg->pose.position.x << ", " 
      << msg->pose.position.y << ")");
    
    // TODO: Plan path using camping_cart_map
    // For now, just publish a simple path with start and goal
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header.frame_id = "map";
    path->header.stamp = this->now();
    
    // Add goal pose to path
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path->header;
    pose.pose = msg->pose;
    path->poses.push_back(pose);
    
    path_pub_->publish(*path);
    RCLCPP_INFO(this->get_logger(), "Path published");
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
