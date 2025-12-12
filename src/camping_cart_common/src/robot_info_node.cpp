#include <rclcpp/rclcpp.hpp>
#include "camping_cart_common/robot_params.hpp"
#include "camping_cart_common/msg/robot_info.hpp"
#include "camping_cart_common/msg/robot_specifications.hpp"
#include "camping_cart_common/msg/sensor_pose.hpp"

using namespace camping_cart;

class RobotInfoNode : public rclcpp::Node
{
public:
  RobotInfoNode()
  : Node("robot_info_node")
  {
    params_ = loadRobotParams(this);

    pub_ = this->create_publisher<camping_cart_common::msg::RobotInfo>(
      "/robot/robot_info", 1);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RobotInfoNode::onTimer, this)
    );
  }

private:
  void onTimer()
  {
    camping_cart_common::msg::RobotInfo msg;

    // ROBOT Specifications
    msg.robot_specifications.wheelbase = params_.wheelbase;
    msg.robot_specifications.track_width = params_.track_width;
    msg.robot_specifications.length = params_.length;
    msg.robot_specifications.width = params_.width;
    msg.robot_specifications.height = params_.height;
    msg.robot_specifications.wheel_radius = params_.wheel_radius;
    msg.robot_specifications.encoder_resolution = params_.encoder_resolution;
    msg.robot_specifications.drive_type = params_.drive_type;

    // IMU
    msg.imu.x = params_.imu.x;
    msg.imu.y = params_.imu.y;
    msg.imu.z = params_.imu.z;
    msg.imu.roll = params_.imu.roll;
    msg.imu.pitch = params_.imu.pitch;
    msg.imu.yaw = params_.imu.yaw;

    // GNSS
    msg.gnss.x = params_.gnss.x;
    msg.gnss.y = params_.gnss.y;
    msg.gnss.z = params_.gnss.z;
    msg.gnss.roll = params_.gnss.roll;
    msg.gnss.pitch = params_.gnss.pitch;
    msg.gnss.yaw = params_.gnss.yaw;

    // LiDAR Front
    msg.lidar_front.x = params_.lidar_front.x;
    msg.lidar_front.y = params_.lidar_front.y;
    msg.lidar_front.z = params_.lidar_front.z;
    msg.lidar_front.roll = params_.lidar_front.roll;
    msg.lidar_front.pitch = params_.lidar_front.pitch;
    msg.lidar_front.yaw = params_.lidar_front.yaw;

    // LiDAR Rear
    msg.lidar_rear.x = params_.lidar_rear.x;
    msg.lidar_rear.y = params_.lidar_rear.y;
    msg.lidar_rear.z = params_.lidar_rear.z;
    msg.lidar_rear.roll = params_.lidar_rear.roll;
    msg.lidar_rear.pitch = params_.lidar_rear.pitch;
    msg.lidar_rear.yaw = params_.lidar_rear.yaw;

    pub_->publish(msg);
  }

  RobotParams params_;
  rclcpp::Publisher<camping_cart_common::msg::RobotInfo>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotInfoNode>());
  rclcpp::shutdown();
  return 0;
}
