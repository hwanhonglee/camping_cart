#include "camping_cart_map/lanelet2_map_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicPoint.h>

namespace camping_cart {
namespace map {

Lanelet2MapNode::Lanelet2MapNode()
  : Node("lanelet2_map_node"), loader_(LoaderConfig())
{
  this->declare_parameter<std::string>("map_path", "");
  this->declare_parameter<double>("origin_lat", 0.0);
  this->declare_parameter<double>("origin_lon", 0.0);
  this->declare_parameter<double>("origin_alt", 0.0);

  config_.map_path = this->get_parameter("map_path").as_string();
  config_.origin_lat = this->get_parameter("origin_lat").as_double();
  config_.origin_lon = this->get_parameter("origin_lon").as_double();
  config_.origin_alt = this->get_parameter("origin_alt").as_double();

  // loader 초기화
  loader_ = Lanelet2MapLoader(config_);

  // map 로드
  loaded_map_ = loader_.load(config_.map_path, true);
  if (!loaded_map_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map!");
    return;
  }

  map_pub_ = this->create_publisher<lanelet::LaneletMap>("/map", 1);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lanelet2map", 1);

  publishMap();
  publishVisualization();
}

void Lanelet2MapNode::initTF()
{
  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
  publishStaticTF();
}

void Lanelet2MapNode::publishMap()
{
  if (!loaded_map_) return;
  map_pub_->publish(*loaded_map_);
}

void Lanelet2MapNode::publishVisualization()
{
  if (!loaded_map_) return;

  visualization_msgs::msg::MarkerArray array_msg;
  int id = 0;

  for (const auto & ll : loaded_map_->laneletLayer) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_id_;
    m.header.stamp = now();
    m.type = m.LINE_STRIP;
    m.action = m.ADD;
    m.ns = "lanelet_centerline";
    m.id = id++;
    m.scale.x = 0.2;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 1.0;

    for (auto & p : ll.centerline()) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = p.z();
      m.points.push_back(gp);
    }

    array_msg.markers.push_back(m);
  }

  viz_pub_->publish(array_msg);
}

void Lanelet2MapNode::publishStaticTF()
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "world";
  tf.child_frame_id = map_frame_id_;
  tf.header.stamp = now();

  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;

  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(tf);
}

} // namespace map
} // namespace camping_cart
