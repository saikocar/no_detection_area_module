# no_detection_area_module

## 概要

`no_detection_area_module` は、ROS 2 ノードであり、Autoware で使用される Lanelet2 マップを基に、特定のレーン属性（例: `no_detection_area`）上に存在し、かつ指定されたラベルに一致する物体を除外する機能を提供します。

---

## 主な機能

* `/map/vector_map` から Lanelet2 マップを受信
* `/objects_in` にパブリッシュされる物体情報を受信
* 以下の条件で物体を除外：

  * 指定サブタイプのレーン（例: `no_detection_area`）上にある
  * ラベルが `excluded_labels` に含まれる
* フィルタ済みの物体を `/objects_out` に出力

---

## 使用方法

### 起動方法

オブジェクトのメッセージの型によって使い分ける必要があります。

autoware_perception_msgs::msg::DetectedObjects

```bash
ros2 launch no_detection_area_module object_exclude_on_lanelet_filter.launch.py
```

autoware_auto_perception_msgs::msg::TrackedObjects

```bash
ros2 launch no_detection_area_module object_exclude_on_lanelet_filter_tracked.launch.py
```

tier4_perception_msgs::msg::DetectedObjectsWithFeature

```bash
ros2 launch no_detection_area_module object_exclude_on_lanelet_filter_withfeature.launch.py
```


### パラメータ

実際の値はconfig/config.yamlから参照して決定します。

| 名前                     | 型        | デフォルト値                     | 説明                           |
| ---------------------- | -------- | -------------------------- | ---------------------------- |
| `map_topic`            | string   | `/map/vector_map`          | HADMapBin メッセージを受信するトピック名    |
| `input_objects_topic`  | string   | `/objects_in`              | 入力物体のトピック名                   |
| `output_objects_topic` | string   | `/objects_out`             | 出力（除外後）物体のトピック名              |
| `target_subtype`       | string   | `no_detection_area`        | 判定対象とする lanelet の subtype 属性 |
| `excluded_labels`      | int64\[] | `[]`                       | 除外対象とする物体のラベル（ID値）           |

---

## トピック

### サブスクライブ

* `pose_topic` (`nav_msgs::msg::Odometry`)
* `map_topic` (`autoware_auto_mapping_msgs/msg/HADMapBin`)
* `input_objects_topic` (`<起動するノード毎のハードコートされた型に依存>`)

### パブリッシュ

* `output_objects_topic` (`<input_objects_topicの型と同じ>`)

---

## 判定ロジック

1. **Laneletのロード**：

   * `subtype` 属性が `target_subtype` と一致する Lanelet を検出
2. **物体形状の処理**：

   * `BOUNDING_BOX` の場合、8隅全てが `target_subtype` 上にあれば「内部」と判定
   * その他の形状は重心のみで判定
3. **除外条件の適用**：

   * 上記で内部と判定された物体のうち、ラベルが `excluded_labels` に含まれていれば除外対象

---

## 依存パッケージ

* `lanelet2_core`, `lanelet2_io`, `lanelet2_projection`, `lanelet2_extension`
* `autoware_auto_mapping_msgs`, `autoware_auto_perception_msgs`, `autoware_perception_msgs`, `tier4_perception_msgs`
* `nav_msgs`, `Eigen`, `boost`

---

