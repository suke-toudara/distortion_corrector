# Point Cloud Distortion Corrector / 点群歪み補正

## 概要 / Overview

**日本語:**

ROS2用の点群歪み補正パッケージです。LiDARのスキャン中のロボット/センサーの動きによって発生する点群の歪みを、IMUまたはOdometryデータを使用して補正します。

**English:**

A ROS2 package for correcting point cloud distortion caused by robot/sensor motion during LiDAR scanning, using IMU or Odometry data.

## 特徴 / Features

- **IMUまたはOdometryデータを使用した点群の歪み補正**
  - ロボットの速度と回転による点群の歪みを正確に補正
  - IMU: 回転のみを補正（角速度データから姿勢を補間）
  - Odom: 位置と回転の両方を補正（位置・姿勢を線形/球面線形補間）
- **SLERP（球面線形補間）による滑らかな回転補間**
- **シンプルで理解しやすいコード構造**
- **Docker環境での簡単な動作確認**
- **標準的なROS2メッセージ型のみを使用**

---

- **Point cloud distortion correction using IMU or Odometry data**
  - Accurately corrects point cloud distortion caused by robot velocity and rotation
  - IMU: Corrects rotation only (interpolates orientation from angular velocity data)
  - Odom: Corrects both position and rotation (linear/spherical linear interpolation)
- **Smooth rotation interpolation using SLERP (Spherical Linear Interpolation)**
- **Simple and understandable code structure**
- **Easy testing with Docker environment**
- **Uses only standard ROS2 message types**

## 動作原理 / How It Works

**日本語:**

LiDARがスキャン中にロボットが移動・回転すると、点群に歪みが発生します。このパッケージは、IMU/Odometryデータを使用して、各点が取得された時刻での姿勢を推定し、すべての点を統一した時刻（スキャン終了時刻）の座標系に補正します。

1. **点群データとIMU/Odomデータを受信**
   - 点群データ（sensor_msgs/PointCloud2）
   - IMUデータ（sensor_msgs/Imu）またはOdomデータ（nav_msgs/Odometry）

2. **各点のタイムスタンプを計算**
   - 点群内の位置から線形補間で各点の取得時刻を推定
   - 例：点群が100msでスキャンされた場合、最初の点は0ms、最後の点は100ms

3. **IMU/Odomデータから姿勢を補間**
   - **IMUの場合**: 角速度データから回転を補間（SLERP）
   - **Odomの場合**: 位置と姿勢を線形/球面線形補間

4. **相対変換を計算して各点を補正**
   - 基準時刻（スキャン終了時刻）の姿勢を取得
   - 各点取得時刻の姿勢を取得
   - 相対変換 = ref_pose^(-1) * point_pose
   - 補正後の点 = 相対変換 * 元の点

5. **補正後の点群をパブリッシュ**

**English:**

When a robot moves or rotates during LiDAR scanning, point cloud distortion occurs. This package uses IMU/Odometry data to estimate the pose at the time each point was captured, and corrects all points to a unified coordinate frame at the reference time (scan end time).

1. **Receive point cloud and IMU/Odom data**
   - Point cloud data (sensor_msgs/PointCloud2)
   - IMU data (sensor_msgs/Imu) or Odom data (nav_msgs/Odometry)

2. **Calculate timestamp for each point**
   - Estimate capture time of each point via linear interpolation based on position in cloud
   - Example: If scan takes 100ms, first point at 0ms, last point at 100ms

3. **Interpolate pose from IMU/Odom data**
   - **IMU case**: Interpolate rotation from angular velocity data (SLERP)
   - **Odom case**: Linear/spherical linear interpolation of position and orientation

4. **Calculate relative transform and correct each point**
   - Get pose at reference time (scan end time)
   - Get pose at each point's capture time
   - Relative transform = ref_pose^(-1) * point_pose
   - Corrected point = relative transform * original point

5. **Publish corrected point cloud**

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
