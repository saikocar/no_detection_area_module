# no_detecttion_area_module
for no detection(ex: car in parking area)

管理方法について考え直す余地があります
mainブランチは`tier4_perception_msgs::msg::DetectedObjectsWithFeature`を利用していますがsaikocarのノードの構造上`autoware_perception_msgs::msg::DetectedObjects`を利用すべきであるという指摘を受け、maintainerも同意しています。
