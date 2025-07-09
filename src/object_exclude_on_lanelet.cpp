#include <rclcpp/rclcpp.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/geometry/Point.h>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <unordered_set>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <iterator>

using std::placeholders::_1;
using lanelet::LaneletMapPtr;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

class ObjectExcludeOnLaneletChecker : public rclcpp::Node, public std::enable_shared_from_this<ObjectExcludeOnLaneletChecker> {
public:
  ObjectExcludeOnLaneletChecker() : Node("object_exclude_on_lanelet_checker") {
    declare_parameter<std::string>("map_topic", "/map/vector_map");
    declare_parameter<std::string>("input_objects_topic", "/objects_in");
    declare_parameter<std::string>("output_objects_topic", "/objects_out");
    declare_parameter<std::string>("marker_topic", "/objects_filtered_marker");
    declare_parameter<std::string>("marker_on_ns", "objects_kept");
    declare_parameter<std::string>("marker_off_ns", "objects_filtered");
    declare_parameter<std::string>("target_subtype", "no_detection_area");
    declare_parameter<std::vector<int64_t>>("excluded_labels", std::vector<int64_t>{});

    map_topic_ = get_parameter("map_topic").as_string();
    input_objects_topic_ = get_parameter("input_objects_topic").as_string();
    output_objects_topic_ = get_parameter("output_objects_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    marker_on_ns_ = get_parameter("marker_on_ns").as_string();
    marker_off_ns_ = get_parameter("marker_off_ns").as_string();
    target_subtype_ = get_parameter("target_subtype").as_string();

    excluded_labels_ = std::unordered_set<int64_t>(
      get_parameter("excluded_labels").as_integer_array().begin(),
      get_parameter("excluded_labels").as_integer_array().end()
    );

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(), std::bind(&ObjectExcludeOnLaneletChecker::mapCallback, this, _1));

    object_sub_ = create_subscription<DetectedObjectsWithFeature>(
      input_objects_topic_, rclcpp::QoS(10), std::bind(&ObjectExcludeOnLaneletChecker::objectCallback, this, _1));

    object_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(output_objects_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr object_sub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unordered_set<int64_t> excluded_labels_;
  lanelet::LaneletMapPtr lanelet_map_;
  std::string map_topic_, input_objects_topic_, output_objects_topic_, marker_topic_;
  std::string marker_on_ns_, marker_off_ns_, target_subtype_;

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg) {
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_);
    RCLCPP_INFO(get_logger(), "Lanelet map loaded with %lu lanelets", lanelet_map_->laneletLayer.size());
  }

  bool isInsideTargetLanelet(const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint2d & pt) const {
    if (!lanelet.hasAttribute("subtype")) return false;
    if (lanelet.attribute("subtype") != target_subtype_) return false;
    return lanelet::geometry::within(pt, lanelet.polygon2d());
  }

  void objectCallback(const DetectedObjectsWithFeature::ConstSharedPtr msg) {
    if (!lanelet_map_) return;

    autoware_auto_perception_msgs::msg::DetectedObjects objects_filtered;
    objects_filtered.header = msg->header;
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    for (const auto& obj_with_feature : msg->feature_objects) {
      const auto & obj = obj_with_feature.object;

      geometry_msgs::msg::PoseStamped input_pose, map_pose;
      input_pose.header = msg->header;
      input_pose.pose = obj.kinematics.pose_with_covariance.pose;

      try {
        map_pose = tf_buffer_->transform(input_pose, "map", tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Transform failed: %s", ex.what());
        continue;
      }

      lanelet::BasicPoint2d pt(map_pose.pose.position.x, map_pose.pose.position.y);
      bool inside_subtype = false;
      for (const auto& lanelet : lanelet_map_->laneletLayer) {
        if (isInsideTargetLanelet(lanelet, pt)) {
          inside_subtype = true;
          break;
        }
      }

      // ラベル除外判定
      bool label_excluded = false;
      if (!obj.classification.empty()) {
        const auto label = static_cast<int64_t>(obj.classification.front().label);
        label_excluded = excluded_labels_.count(label) > 0;
      }

      if (!inside_subtype || !label_excluded) {
        objects_filtered.objects.push_back(obj);
      }

      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = (!inside_subtype || !label_excluded) ? marker_off_ns_ : marker_on_ns_;
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = input_pose.pose;
      marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
      marker.color.a = 0.8;
      if (!inside_subtype || !label_excluded) {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
      } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      }
      markers.markers.push_back(marker);
    }

    object_pub_->publish(objects_filtered);
    marker_pub_->publish(markers);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectExcludeOnLaneletChecker>());
  rclcpp::shutdown();
  return 0;
}
