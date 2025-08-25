#include <rclcpp/rclcpp.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/geometry/Point.h>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <unordered_set>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  std::vector<int64_t> excluded_labels_;
  std::vector<lanelet::ConstLanelet> lanelet_map_;
  std::vector<lanelet::ConstLanelet> lanelet_map_near_;
  std::string map_topic_, input_objects_topic_, output_objects_topic_,pose_topic_;
  std::string target_subtype_;
  nav_msgs::msg::Odometry current_pos_;
  lanelet::BasicPoint2d current_pos;
  bool current_pos_initialized = false;
public:
  ObjectExcludeOnLaneletChecker(const rclcpp::NodeOptions &options) : Node("object_exclude_on_lanelet_checker", options)
  {
    declare_parameter<std::string>("map_topic", "/map/vector_map");
    declare_parameter<std::string>("input_objects_topic", "/objects_in");
    declare_parameter<std::string>("output_objects_topic", "/objects_out");
    declare_parameter<std::string>("target_subtype", "no_detection_area");
    declare_parameter<std::string>("pose_topic", "/localization/kinematic_state");
    declare_parameter<std::vector<int64_t>>("excluded_labels", std::vector<int64_t>{});

    map_topic_ = get_parameter("map_topic").as_string();
    input_objects_topic_ = get_parameter("input_objects_topic").as_string();
    output_objects_topic_ = get_parameter("output_objects_topic").as_string();
    target_subtype_ = get_parameter("target_subtype").as_string();
    pose_topic_ = get_parameter("pose_topic").as_string();
    excluded_labels_ = get_parameter("excluded_labels").as_integer_array();

    map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
     map_topic_, rclcpp::QoS(1).transient_local().reliable(), std::bind(&ObjectExcludeOnLaneletChecker::mapCallback, this, _1));

    object_sub_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
     input_objects_topic_, rclcpp::QoS(10), std::bind(&ObjectExcludeOnLaneletChecker::objectCallback, this, _1));

    object_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(output_objects_topic_, 10);

    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, rclcpp::QoS(10),std::bind(&ObjectExcludeOnLaneletChecker::poseCallback, this, _1));
  }

private:
  void poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg){
      if(msg->header.frame_id != "map")
      {
        RCLCPP_INFO(get_logger(), "pose frame is %s, but this module expects msg frame", msg->header.frame_id.c_str());
        lanelet_map_.clear();
        current_pos_initialized = false;
        return;
      }
    current_pos_ = *msg;
    current_pos_initialized = true;
  }
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg) {
    try {
      if(msg.header.frame_id != "map")
      {
        RCLCPP_INFO(get_logger(), "lanelet map frame is %s, but this module expects msg frame", msg.header.frame_id.c_str());
        lanelet_map_.clear();
        return;
      }
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
  bool isInsideTargetLanelet(const lanelet::BasicPoint2d & pt) const {
    for (const auto & lanelet : lanelet_map_near_) {
      if (!lanelet::geometry::within(pt, lanelet.polygon2d())) {return false;}
    }
    return true;
  }

  void objectCallback(const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg) {
    //if (!lanelet_map_) return;
    if (lanelet_map_.empty() || !current_pos_initialized){
      object_pub_->publish(*msg);
      return;
    }
    current_pos = { current_pos_.pose.pose.position.x,
                    current_pos_.pose.pose.position.y };

    double threshold = 50.0; //50[m]以内なら近いと判定
    lanelet_map_near_.clear();
    for (const auto & ll : lanelet_map_) { // std::vector<ConstLanelet>
        double dist = lanelet::geometry::distance2d(current_pos, ll.polygon2d()); // 点とレーンの距離[m]
        //距離内ならnearに入れて以降の処理に利用する
        if (dist < threshold) {lanelet_map_near_.push_back(ll);}         
    }
    autoware_perception_msgs::msg::DetectedObjects objects_filtered;
    objects_filtered.header = msg->header;

    for (const auto& obj : msg->objects) {
      geometry_msgs::msg::PoseStamped input_pose, map_pose;
      input_pose.header = msg->header;
      input_pose.pose = obj.kinematics.pose_with_covariance.pose;
      if(input_pose.header.frame_id != "map"){
        RCLCPP_INFO(get_logger(), "object frame is %s, but this module expects msg frame,so the object cannot be excluded", input_pose.header.frame_id.c_str());
         objects_filtered.objects.push_back(obj);
         continue;
      }

      bool inside_subtype = true;
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
        Eigen::Quaterniond q(input_pose.pose.orientation.w,
                            input_pose.pose.orientation.x,
                            input_pose.pose.orientation.y,
                            input_pose.pose.orientation.z);
        Eigen::Vector3d trans(input_pose.pose.position.x,
                              input_pose.pose.position.y,
                              input_pose.pose.position.z);
        for (const auto& corner_local : corners_local) {
            Eigen::Vector3d corner_world = q * corner_local + trans;
            lanelet::BasicPoint2d pt(corner_world.x(), corner_world.y());
            if (!isInsideTargetLanelet(pt)) {inside_subtype = false;break;}
        }
      } else {
        // --- 重心のみで判定 ---
        lanelet::BasicPoint2d pt(input_pose.pose.position.x, input_pose.pose.position.y);
        if (!isInsideTargetLanelet(pt)) {inside_subtype = false;}
      }
      // ラベル除外判定
      bool label_excluded = false;
      try{
      if (!obj.classification.empty()) {
        const auto label = static_cast<int64_t>(obj.classification.front().label);
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
    }

    object_pub_->publish(objects_filtered);
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
