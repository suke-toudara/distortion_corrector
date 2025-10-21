# Point Cloud Distortion Corrector / 点群歪み補正

## 概要 / Overview

**日本語:**

ROS2用の点群歪み補正パッケージです。LiDARのスキャン中のロボット/センサーの動きによって発生する点群の歪みを、IMUまたはOdometryデータを使用して補正します。

**English:**

A ROS2 package for correcting point cloud distortion caused by robot/sensor motion during LiDAR scanning, using IMU or Odometry data.

## 特徴 / Features

- IMUまたはOdometryデータを使用した点群の歪み補正
- TF2を使用した正確な座標変換
- シンプルで理解しやすいコード構造
- Docker環境での簡単な動作確認
- 標準的なROS2メッセージ型のみを使用

---

- Point cloud distortion correction using IMU or Odometry data
- Accurate coordinate transformation using TF2
- Simple and understandable code structure
- Easy testing with Docker environment
- Uses only standard ROS2 message types

## 動作原理 / How It Works

**日本語:**

1. 点群データ、IMU/Odomデータを受信
2. 各点のタイムスタンプを計算（点群内の位置から線形補間）
3. 各点のタイムスタンプでのセンサー位置・姿勢をTF2から取得
4. 各点を基準時刻の座標系に変換して歪みを補正
5. 補正後の点群をパブリッシュ

**English:**

1. Receive point cloud and IMU/Odom data
2. Calculate timestamp for each point (linear interpolation based on position in cloud)
3. Get sensor pose at each point's timestamp from TF2
4. Transform each point to reference frame to correct distortion
5. Publish corrected point cloud

## 必要要件 / Requirements

- ROS2 Humble
- PCL (Point Cloud Library)
- TF2

## ディレクトリ構造 / Directory Structure

```
distortion_corrector/
├── include/
│   └── distortion_corrector/
│       └── distortion_corrector_node.hpp
├── src/
│   ├── distortion_corrector_node.cpp
│   └── distortion_corrector_main.cpp
├── launch/
│   └── distortion_corrector.launch.py
├── config/
│   └── distortion_corrector.yaml
├── CMakeLists.txt
├── package.xml
├── Dockerfile
├── docker-compose.yml
└── README.md
```

## ビルド方法 / Build Instructions

### ローカル環境 / Local Environment

```bash
# ワークスペース作成
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repository>

# ビルド
cd ~/ros2_ws
colcon build --packages-select distortion_corrector
source install/setup.bash
```

### Docker環境 / Docker Environment

```bash
# ビルド
docker-compose build

# 起動（別ターミナルで実行）
docker-compose up
```

## 使用方法 / Usage

### 基本起動 / Basic Launch

```bash
# IMUを使用
ros2 launch distortion_corrector distortion_corrector.launch.py use_data_source:=imu

# Odomを使用
ros2 launch distortion_corrector distortion_corrector.launch.py use_data_source:=odom
```

### ノード単体起動 / Standalone Node

```bash
ros2 run distortion_corrector distortion_corrector_main \
  --ros-args \
  -p use_data_source:=imu \
  -p base_frame:=base_link
```

## トピック / Topics

### Subscribed

- `~/input/pointcloud` (sensor_msgs/PointCloud2): 入力点群 / Input point cloud
- `~/input/imu` (sensor_msgs/Imu): IMUデータ / IMU data
- `~/input/odom` (nav_msgs/Odometry): Odometryデータ / Odometry data

### Published

- `~/output/pointcloud` (sensor_msgs/PointCloud2): 補正後の点群 / Corrected point cloud

## パラメータ / Parameters

| Parameter | Type | Default | Description (日本語) | Description (English) |
|-----------|------|---------|---------------------|----------------------|
| `use_data_source` | string | "imu" | 補正に使用するデータソース（"imu" または "odom"） | Data source for correction ("imu" or "odom") |
| `base_frame` | string | "base_link" | 基準座標フレーム | Base coordinate frame |
| `queue_size` | double | 2.0 | データキューのサイズ（秒） | Data queue size (seconds) |
| `max_queue_size` | int | 200 | データキューの最大メッセージ数 | Maximum number of messages in queue |

## 設定ファイルのカスタマイズ / Configuration Customization

`config/distortion_corrector.yaml`を編集してパラメータを変更できます:

```yaml
/**:
  ros__parameters:
    base_frame: "base_link"
    use_data_source: "imu"  # "imu" or "odom"
    queue_size: 2.0
    max_queue_size: 200
```

## トラブルシューティング / Troubleshooting

### 補正が動作しない / Correction Not Working

**日本語:**

1. TFツリーが正しく構成されているか確認:
   ```bash
   ros2 run tf2_tools view_frames
   ```

2. IMU/Odomデータが届いているか確認:
   ```bash
   ros2 topic echo /imu/data
   ros2 topic echo /odom
   ```

3. 点群データが届いているか確認:
   ```bash
   ros2 topic echo /velodyne_points
   ```

**English:**

1. Check if TF tree is properly configured:
   ```bash
   ros2 run tf2_tools view_frames
   ```

2. Check if IMU/Odom data is being published:
   ```bash
   ros2 topic echo /imu/data
   ros2 topic echo /odom
   ```

3. Check if point cloud data is being published:
   ```bash
   ros2 topic echo /velodyne_points
   ```

## 参考資料 / References

- [Autoware Universe - Distortion Corrector](https://github.com/autowarefoundation/autoware_universe/tree/main/sensing/autoware_pointcloud_preprocessor)
- [Livox Cloud Undistortion](https://github.com/Livox-SDK/livox_cloud_undistortion)

## ライセンス / License

Apache License 2.0

## 作者 / Author

Created with Claude Code
