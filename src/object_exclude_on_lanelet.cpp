#include <rclcpp/rclcpp.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/geometry/Point.h>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <unordered_set>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <Eigen/Geometry>  // Eigen::Vector3d, Eigen::Quaterniond

#include <iterator>

using std::placeholders::_1;
using lanelet::LaneletMapPtr;

class ObjectExcludeOnLaneletChecker : public rclcpp::Node, public std::enable_shared_from_this<ObjectExcludeOnLaneletChecker> {
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  //std::unordered_set<int64_t> excluded_labels_;
  std::vector<int64_t> excluded_labels_;
  //lanelet::LaneletMapPtr lanelet_map_;
  std::vector<lanelet::ConstLanelet> lanelet_map_;
  std::string map_topic_, input_objects_topic_, output_objects_topic_, marker_topic_;
  std::string marker_on_ns_, marker_off_ns_, target_subtype_;
 
public:
  ObjectExcludeOnLaneletChecker(const rclcpp::NodeOptions &options) : Node("object_exclude_on_lanelet_checker", options)
    , tf_buffer_(this->get_clock())
    , tf_listener_(tf_buffer_)
  {
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

    /*excluded_labels_ = std::unordered_set<int64_t>(
      get_parameter("excluded_labels").as_integer_array().begin(),
      get_parameter("excluded_labels").as_integer_array().end()
    );*/
    excluded_labels_ = get_parameter("excluded_labels").as_integer_array();

    map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
     map_topic_, rclcpp::QoS(1).transient_local().reliable(), std::bind(&ObjectExcludeOnLaneletChecker::mapCallback, this, _1));

    object_sub_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
     input_objects_topic_, rclcpp::QoS(10), std::bind(&ObjectExcludeOnLaneletChecker::objectCallback, this, _1));

    object_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(output_objects_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  }

private:
  /*
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg) {
    try{
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_);
    RCLCPP_INFO(get_logger(), "Lanelet map loaded with %lu lanelets", lanelet_map_->laneletLayer.size());}
    catch(const std::exception & e){
    RCLCPP_INFO(get_logger(), "map load failed");
    lanelet_map_=nullptr;
    }
  }
  */

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg) {
    try {
      auto full_map = std::make_shared<lanelet::LaneletMap>();
      lanelet::utils::conversion::fromBinMsg(msg, full_map);

      RCLCPP_INFO(get_logger(), "Full lanelet map loaded with %lu lanelets", full_map->laneletLayer.size());

      // target subtype の lanelet のみを保持
      lanelet_map_.clear();
      for (const auto & lanelet : full_map->laneletLayer) {
        if (!lanelet.hasAttribute("subtype")) continue;
        if (lanelet.attribute("subtype") != target_subtype_) continue;
        lanelet_map_.push_back(lanelet);
      }

      RCLCPP_INFO(get_logger(), "Filtered lanelets with subtype='%s': %lu",
                  target_subtype_.c_str(), lanelet_map_.size());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Map load failed: %s", e.what());
      lanelet_map_.clear();
    }
  }
  /*
  bool isInsideTargetLanelet(const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint2d & pt) const {
    if (!lanelet.hasAttribute("subtype")) return false;
    if (lanelet.attribute("subtype") != target_subtype_) return false;
    return lanelet::geometry::within(pt, lanelet.polygon2d());
  }
  */
  bool isInsideTargetLanelet(const lanelet::BasicPoint2d & pt) const {
    for (const auto & lanelet : lanelet_map_) {
      if (lanelet::geometry::within(pt, lanelet.polygon2d())) {
        return true;
      }
    }
    return false;
  }

  void objectCallback(const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg) {
    //if (!lanelet_map_) return;
    if (lanelet_map_.empty()){
      object_pub_->publish(*msg);
      RCLCPP_INFO(get_logger(), "No no-detection-area or map-data is not arrived, so marker for rviz is not making");
      return;
    }

    autoware_perception_msgs::msg::DetectedObjects objects_filtered;
    objects_filtered.header = msg->header;
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;

    for (const auto& obj : msg->objects) {
      geometry_msgs::msg::PoseStamped input_pose, map_pose;
      input_pose.header = msg->header;
      input_pose.pose = obj.kinematics.pose_with_covariance.pose;

      try {
        map_pose = tf_buffer_.transform(input_pose, "map", tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Transform failed: %s", ex.what());
        continue;
      }

      bool inside_subtype = false;
      if (obj.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
        // --- BOX の8隅を使った判定 ---
        const auto& dims = obj.shape.dimensions;
        const double x = dims.x / 2.0;
        const double y = dims.y / 2.0;
        const double z = dims.z / 2.0;

        // ローカル8隅座標（XY平面のみ使う）
        std::vector<Eigen::Vector3d> corners_local = {
          { x,  y,  z}, { x, -y,  z}, {-x,  y,  z}, {-x, -y,  z},
          { x,  y, -z}, { x, -y, -z}, {-x,  y, -z}, {-x, -y, -z}
        };

        // poseの回転と位置を行列に変換
        Eigen::Quaterniond q(map_pose.pose.orientation.w,
                            map_pose.pose.orientation.x,
                            map_pose.pose.orientation.y,
                            map_pose.pose.orientation.z);
        Eigen::Vector3d trans(map_pose.pose.position.x,
                              map_pose.pose.position.y,
                              map_pose.pose.position.z);
        for (const auto& corner_local : corners_local) {
          try{
          Eigen::Vector3d corner_world = q * corner_local + trans;
          lanelet::BasicPoint2d pt(corner_world.x(), corner_world.y());
          //for (const auto& lanelet : lanelet_map_->laneletLayer) {
          for (const auto & lanelet : lanelet_map_) {
            //if (isInsideTargetLanelet(lanelet, pt)) {
            if (isInsideTargetLanelet(pt)) {
              inside_subtype = true;
              break;
            }
          }}catch(const std::exception & e){
            RCLCPP_WARN(get_logger(), "layer failed");
            continue;
          }
          if (inside_subtype) break; // 一つでも入っていればよい
        }
      } else {
        // --- 重心のみで判定 ---
        try{
        lanelet::BasicPoint2d pt(map_pose.pose.position.x, map_pose.pose.position.y);
        //for (const auto& lanelet : lanelet_map_->laneletLayer) {
        for (const auto & lanelet : lanelet_map_) {
          //if (isInsideTargetLanelet(lanelet, pt)) {
          if (isInsideTargetLanelet(pt)) {
            inside_subtype = true;
            break;
          }
        }
      }catch(const std::exception & e){
            RCLCPP_WARN(get_logger(), "gravity failed");
            continue;        
      }
      }
      // ラベル除外判定
      bool label_excluded = false;
      try{
      if (!obj.classification.empty()) {
        const auto label = static_cast<int64_t>(obj.classification.front().label);
        //label_excluded = excluded_labels_.count(label) > 0;
        for(int label_len=0;label_len<excluded_labels_.size();label_len++){
          if(excluded_labels_[label_len]==label){label_excluded=true;break;}
        }
      }

      if (!inside_subtype || !label_excluded) {
        objects_filtered.objects.push_back(obj);
      }
    }catch(const std::exception & e){
            RCLCPP_WARN(get_logger(), "label exclude failed");
            continue;        
    }
    try{

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
    }catch(const std::exception & e){
            RCLCPP_WARN(get_logger(), "make marker failed");
            continue;        
    }
    }

    object_pub_->publish(objects_filtered);
    marker_pub_->publish(markers);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ObjectExcludeOnLaneletChecker>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
