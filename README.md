# no_detection_area_module

## 概要

`no_detection_area_module` は、ROS 2 ノードであり、Autoware で使用される Lanelet2 マップを基に、特定のレーン属性（例: `no_detection_area`）上に存在し、かつ指定されたラベルに一致する物体を除外する機能を提供します。デバッグ用に除外された/されなかった物体について、デバッグ可視化のためのRVizマーカーも生成します。

---

## 主な機能

* `/map/vector_map` から Lanelet2 マップを受信
* `/objects_in` にパブリッシュされる物体情報を受信
* 以下の条件で物体を除外：

  * 指定サブタイプのレーン（例: `no_detection_area`）上にある
  * ラベルが `excluded_labels` に含まれる
* フィルタ済みの物体を `/objects_out` に出力
* 除外された/されなかった物体の位置を `/objects_filtered_marker` として RViz に可視化

---

## 使用方法

### 起動方法

```bash
ros2 launch no_detection_area_module object_exclude_on_lanelet_filter.launch.py
```

### パラメータ

実際の値はconfig/config.yamlから参照して決定します。

| 名前                     | 型        | デフォルト値                     | 説明                           |
| ---------------------- | -------- | -------------------------- | ---------------------------- |
| `map_topic`            | string   | `/map/vector_map`          | HADMapBin メッセージを受信するトピック名    |
| `input_objects_topic`  | string   | `/objects_in`              | 入力物体のトピック名                   |
| `output_objects_topic` | string   | `/objects_out`             | 出力（除外後）物体のトピック名              |
| `marker_topic`         | string   | `/objects_filtered_marker` | 可視化マーカートピック名                 |
| `marker_on_ns`         | string   | `objects_kept`             | 除外対象でない物体のマーカー名前空間           |
| `marker_off_ns`        | string   | `objects_filtered`         | 除外対象のマーカー名前空間                |
| `target_subtype`       | string   | `no_detection_area`        | 判定対象とする lanelet の subtype 属性 |
| `excluded_labels`      | int64\[] | `[]`                       | 除外対象とする物体のラベル（ID値）           |

---

## トピック

### サブスクライブ

* `map_topic` (`autoware_auto_mapping_msgs/msg/HADMapBin`)
* `input_objects_topic` (`autoware_auto_perception_msgs/msg/DetectedObjects`)

### パブリッシュ

* `output_objects_topic` (`autoware_auto_perception_msgs/msg/DetectedObjects`)
* `marker_topic` (`visualization_msgs/msg/MarkerArray`)

---

## 判定ロジック

1. **Laneletのロード**：

   * `subtype` 属性が `target_subtype` と一致する Lanelet を検出
2. **物体形状の処理**：

   * `BOUNDING_BOX` の場合、8隅のうち1点でも `target_subtype` 上にあれば「内部」と判定
   * その他の形状は重心のみで判定
3. **除外条件の適用**：

   * 上記で内部と判定された物体のうち、ラベルが `excluded_labels` に含まれていれば除外対象

---

## 可視化（RViz）

* 除外された物体は `marker_on_ns`（赤）
* 除外されなかった物体は `marker_off_ns`（青）

---

## 依存パッケージ

* `lanelet2_core`, `lanelet2_io`, `lanelet2_projection`, `lanelet2_extension`
* `autoware_auto_mapping_msgs`, `autoware_auto_perception_msgs`
* `tf2_ros`, `visualization_msgs`, `Eigen`, `boost`

---

