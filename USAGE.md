# 使用例 / Usage Examples

## 1. Dockerで起動 / Running with Docker

### 基本的な起動 / Basic Usage

```bash
# スクリプトを使用（推奨）
./scripts/run_docker.sh

# または手動で
xhost +local:docker
docker-compose up --build
```

### コンテナ内でコマンド実行 / Execute Commands Inside Container

```bash
# コンテナに入る
docker exec -it distortion_corrector bash

# ノードを手動起動
ros2 run distortion_corrector distortion_corrector_main

# トピック確認
ros2 topic list
ros2 topic echo /corrected_points
```

## 2. ローカル環境での起動 / Running Locally

### ビルド / Build

```bash
cd ~/ros2_ws
colcon build --packages-select distortion_corrector --symlink-install
source install/setup.bash
```

### 起動 / Launch

#### IMUを使用 / Using IMU

```bash
ros2 launch distortion_corrector distortion_corrector.launch.py \
  use_data_source:=imu
```

#### Odometryを使用 / Using Odometry

```bash
ros2 launch distortion_corrector distortion_corrector.launch.py \
  use_data_source:=odom \
  base_frame:=base_link
```

## 3. カスタムトピックマッピング / Custom Topic Remapping

### 例1: Velodyneセンサー / Example 1: Velodyne Sensor

```bash
ros2 run distortion_corrector distortion_corrector_main \
  --ros-args \
  -r ~/input/pointcloud:=/velodyne_points \
  -r ~/input/imu:=/imu/data \
  -r ~/output/pointcloud:=/velodyne_points_corrected \
  -p use_data_source:=imu \
  -p base_frame:=base_link
```

### 例2: Livoxセンサー / Example 2: Livox Sensor

```bash
ros2 run distortion_corrector distortion_corrector_main \
  --ros-args \
  -r ~/input/pointcloud:=/livox/lidar \
  -r ~/input/imu:=/livox/imu \
  -r ~/output/pointcloud:=/livox/lidar_corrected \
  -p use_data_source:=imu \
  -p base_frame:=livox_frame
```

### 例3: Odometryを使用 / Example 3: Using Odometry

```bash
ros2 run distortion_corrector distortion_corrector_main \
  --ros-args \
  -r ~/input/pointcloud:=/points \
  -r ~/input/odom:=/odom \
  -r ~/output/pointcloud:=/points_corrected \
  -p use_data_source:=odom \
  -p base_frame:=base_link
```

## 4. パラメータファイルを使用 / Using Parameter File

カスタムパラメータファイルを作成:

```yaml
# my_params.yaml
/**:
  ros__parameters:
    base_frame: "base_link"
    use_data_source: "imu"
    queue_size: 3.0
    max_queue_size: 300
```

起動:

```bash
ros2 run distortion_corrector distortion_corrector_main \
  --ros-args --params-file my_params.yaml
```

## 5. RViz2で可視化 / Visualization with RViz2

### 別ターミナルでRViz起動 / Launch RViz in Separate Terminal

```bash
rviz2
```

RViz2で以下を設定:
1. Fixed Frame: `base_link`
2. Add → PointCloud2
3. Topic: `/corrected_points` (補正後)
4. Topic: `/velodyne_points` (補正前、比較用)

## 6. トラブルシューティング / Troubleshooting

### TFツリーの確認 / Check TF Tree

```bash
# TFツリーを可視化
ros2 run tf2_tools view_frames

# 特定のTFを確認
ros2 run tf2_ros tf2_echo base_link sensor_frame
```

### トピックの確認 / Check Topics

```bash
# トピック一覧
ros2 topic list

# トピックの詳細情報
ros2 topic info /velodyne_points

# トピックのデータ確認
ros2 topic echo /velodyne_points --no-arr

# トピックのHz確認
ros2 topic hz /velodyne_points
```

### ノードの確認 / Check Node

```bash
# ノード一覧
ros2 node list

# ノードの情報
ros2 node info /distortion_corrector

# パラメータ確認
ros2 param list /distortion_corrector
ros2 param get /distortion_corrector use_data_source
```

### ログの確認 / Check Logs

```bash
# ノードのログ
ros2 run distortion_corrector distortion_corrector_main \
  --ros-args --log-level debug
```

## 7. 他のノードとの連携例 / Integration Examples

### Autowareとの連携 / Integration with Autoware

```bash
# Autowareの他のノードと一緒に起動
ros2 launch your_package autoware_with_distortion_correction.launch.py
```

### ROSbagから再生 / Playback from ROSbag

```bash
# Terminal 1: ROSbag再生
ros2 bag play your_data.db3

# Terminal 2: 補正ノード起動
ros2 launch distortion_corrector distortion_corrector.launch.py

# Terminal 3: 結果を記録
ros2 bag record /corrected_points
```

## 8. パフォーマンスチューニング / Performance Tuning

### 高頻度データの場合 / For High-Frequency Data

```yaml
/**:
  ros__parameters:
    queue_size: 1.0  # より短いキュー
    max_queue_size: 100  # 少ないキューサイズ
```

### 低頻度データの場合 / For Low-Frequency Data

```yaml
/**:
  ros__parameters:
    queue_size: 5.0  # より長いキュー
    max_queue_size: 500  # 大きいキューサイズ
```

## 9. 複数センサーの場合 / Multiple Sensors

```bash
# センサー1
ros2 run distortion_corrector distortion_corrector_main \
  --ros-args \
  -r __node:=corrector_front \
  -r ~/input/pointcloud:=/front/points \
  -r ~/output/pointcloud:=/front/points_corrected

# センサー2
ros2 run distortion_corrector distortion_corrector_main \
  --ros-args \
  -r __node:=corrector_rear \
  -r ~/input/pointcloud:=/rear/points \
  -r ~/output/pointcloud:=/rear/points_corrected
```
