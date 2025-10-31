# Point Cloud Distortion Corrector / 点群歪み補正

## 概要 / Overview

**日本語:**

ROS2用の点群歪み補正パッケージです。LiDARのスキャン中のロボット/センサーの動きによって発生する点群の歪みを、**IMUで回転方向、Odometryで並進方向**を補正します。

Livox SDK方式のデータ同期機構（`mutex` + `condition_variable`）を採用し、データが揃った時点で補正処理を実行します。

**English:**

A ROS2 package for correcting point cloud distortion caused by robot/sensor motion during LiDAR scanning. **IMU corrects rotation, Odometry corrects translation.**

Implements Livox SDK-style data synchronization using `mutex` + `condition_variable` to process point clouds when all data is ready.

---

## 特徴 / Features

**日本語:**

- ✅ **IMUで回転方向の歪みを補正** - SLERP（球面線形補間）による滑らかな回転補間
- ✅ **Odometryで並進方向の歪みを補正** - 線形補間による位置補正
- ✅ **Livox方式のデータ同期** - `mtx_buffer_.unlock()` + `sig_buffer_.notify_all()` でデータが揃ったら処理
- ✅ **マルチスレッド処理** - メインスレッドでデータ受信、別スレッドで補正処理
- ✅ **一般的なLiDAR対応** - Velodyne、Ouster、Hesai等の回転型LiDARに対応
- ✅ **標準ROS2メッセージのみ使用** - 独自メッセージ型不要

**English:**

- ✅ **IMU corrects rotational distortion** - Smooth rotation interpolation using SLERP
- ✅ **Odometry corrects translational distortion** - Linear interpolation for position
- ✅ **Livox-style data synchronization** - Process when data is ready using `mtx_buffer_.unlock()` + `sig_buffer_.notify_all()`
- ✅ **Multi-threaded processing** - Main thread for data reception, separate thread for correction
- ✅ **General LiDAR support** - Works with Velodyne, Ouster, Hesai, and other rotating LiDARs
- ✅ **Standard ROS2 messages only** - No custom message types required

---

## 動作原理 / How It Works

### アーキテクチャ / Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Main Thread (Callbacks)                  │
├─────────────────────────────────────────────────────────────┤
│  PointCloud Callback  │  IMU Callback  │  Odom Callback     │
│         ↓             │       ↓        │        ↓           │
│  cloud_buffer_        │  imu_buffer_   │  odom_buffer_      │
│         ↓             │       ↓        │        ↓           │
│         └─────────────┴────────────────┴─────────┘          │
│                          ↓                                   │
│                  mtx_buffer_.unlock()                        │
│                  sig_buffer_.notify_all()  ← Livox方式      │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│              Processing Thread (Background)                  │
├─────────────────────────────────────────────────────────────┤
│  sig_buffer_.wait() ← データが揃うまで待機                  │
│         ↓                                                    │
│  データが揃ったら補正処理を実行                             │
│         ↓                                                    │
│  1. IMUから回転を補間 (SLERP)                               │
│  2. Odomから位置を補間 (線形補間)                           │
│  3. 各点に相対変換を適用                                    │
│         ↓                                                    │
│  補正後の点群をパブリッシュ                                 │
└─────────────────────────────────────────────────────────────┘
```

### 補正アルゴリズム / Correction Algorithm

**日本語:**

LiDARがスキャン中（例：100ms）にロボットが移動・回転すると、点群に歪みが発生します。

1. **データ受信とバッファリング**
   - PointCloud、IMU、Odomデータをそれぞれのバッファに蓄積
   - データ受信時に `sig_buffer_.notify_all()` で処理スレッドに通知

2. **処理スレッドでの待機**
   - `sig_buffer_.wait()` でデータが揃うまで待機
   - 点群データとIMU/Odomデータが十分揃ったら処理開始

3. **各点のタイムスタンプ計算**
   - 点群内の位置から線形補間で各点の取得時刻を推定
   - 例：100ms スキャンの場合、最初の点は0ms、最後の点は100ms

4. **IMUから回転を補間**
   - 各点の取得時刻でのIMU姿勢をSLERP補間
   - 回転のみを補正（IMUは位置情報を持たない）

5. **Odomから位置を補間**
   - 各点の取得時刻でのOdom位置を線形補間
   - 並進のみを補正

6. **相対変換を適用**
   ```
   # 基準時刻の姿勢を取得
   ref_rotation = IMUから補間(基準時刻)
   ref_translation = Odomから補間(基準時刻)

   # 各点について
   for each point:
     # 点取得時刻の姿勢を取得
     point_rotation = IMUから補間(点の時刻)
     point_translation = Odomから補間(点の時刻)

     # ワールド座標系に変換
     pt_world = point_rotation * pt + point_translation

     # 基準時刻のセンサー座標系に変換
     pt_corrected = ref_rotation^(-1) * (pt_world - ref_translation)
   ```

7. **補正後の点群をパブリッシュ**

**English:**

When a robot moves or rotates during LiDAR scanning (e.g., 100ms), point cloud distortion occurs.

1. **Data Reception and Buffering**
   - Accumulate PointCloud, IMU, Odom data in respective buffers
   - Notify processing thread with `sig_buffer_.notify_all()` on data reception

2. **Processing Thread Waits**
   - Wait using `sig_buffer_.wait()` until data is ready
   - Start processing when sufficient point cloud and IMU/Odom data available

3. **Calculate Point Timestamps**
   - Estimate capture time via linear interpolation based on position in cloud
   - Example: For 100ms scan, first point at 0ms, last point at 100ms

4. **Interpolate Rotation from IMU**
   - SLERP interpolation of IMU orientation at each point's capture time
   - Corrects rotation only (IMU has no position information)

5. **Interpolate Translation from Odom**
   - Linear interpolation of Odom position at each point's capture time
   - Corrects translation only

6. **Apply Relative Transform**
   ```
   # Get reference pose
   ref_rotation = interpolate_IMU(reference_time)
   ref_translation = interpolate_Odom(reference_time)

   # For each point
   for each point:
     # Get pose at point capture time
     point_rotation = interpolate_IMU(point_time)
     point_translation = interpolate_Odom(point_time)

     # Transform to world frame
     pt_world = point_rotation * pt + point_translation

     # Transform to sensor frame at reference time
     pt_corrected = ref_rotation^(-1) * (pt_world - ref_translation)
   ```

7. **Publish Corrected Point Cloud**

---

## 必要要件 / Requirements

- ROS2 Humble (or later)
- PCL (Point Cloud Library)
- Eigen3

---

## ビルド / Build

```bash
# ワークスペース作成
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repository>

# ビルド
cd ~/ros2_ws
colcon build --packages-select distortion_corrector --symlink-install
source install/setup.bash
```

---

## 使用方法 / Usage

### 基本起動 / Basic Launch

```bash
# IMUとOdom両方を使用（デフォルト）
ros2 launch distortion_corrector distortion_corrector.launch.py

# IMUのみ使用（回転のみ補正）
ros2 launch distortion_corrector distortion_corrector.launch.py use_imu:=true use_odom:=false

# Odomのみ使用（並進のみ補正）
ros2 launch distortion_corrector distortion_corrector.launch.py use_imu:=false use_odom:=true
```

### トピック / Topics

#### Subscribed

- `~/input/pointcloud` (sensor_msgs/PointCloud2): 入力点群 / Input point cloud
- `~/input/imu` (sensor_msgs/Imu): IMUデータ（回転補正用） / IMU data (for rotation correction)
- `~/input/odom` (nav_msgs/Odometry): Odometryデータ（並進補正用） / Odom data (for translation correction)

#### Published

- `~/output/pointcloud` (sensor_msgs/PointCloud2): 補正後の点群 / Corrected point cloud

### パラメータ / Parameters

| Parameter | Type | Default | Description (日本語) | Description (English) |
|-----------|------|---------|---------------------|----------------------|
| `scan_duration` | double | 0.1 | スキャン時間（秒） | Scan duration (seconds) |
| `max_buffer_size` | int | 100 | IMU/Odomバッファの最大サイズ | Max buffer size for IMU/Odom |
| `use_imu` | bool | true | IMUで回転補正を有効化 | Enable IMU for rotation correction |
| `use_odom` | bool | true | Odomで並進補正を有効化 | Enable Odom for translation correction |

---

## Docker環境 / Docker Environment

```bash
# ビルドと起動
docker-compose up --build

# または
./scripts/run_docker.sh
```

---

## Livox方式のデータ同期 / Livox-Style Data Synchronization

**日本語:**

このパッケージは、Livox SDKの点群補正コードと同様のデータ同期機構を採用しています：

```cpp
// Callback関数（メインスレッド）
void pointCloudCallback(const PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_buffer_);
  cloud_buffer_.push_back(msg);

  // 処理スレッドに通知（Livox方式）
  sig_buffer_.notify_all();
}

// 処理スレッド（バックグラウンド）
void processingLoop() {
  while (!shutdown_) {
    std::unique_lock<std::mutex> lock(mtx_buffer_);

    // データが揃うまで待機
    sig_buffer_.wait(lock, [this] {
      return shutdown_ || !cloud_buffer_.empty();
    });

    // データ取得
    auto cloud_msg = cloud_buffer_.front();
    cloud_buffer_.pop_front();

    // ロック解除してから処理（Livox方式）
    lock.unlock();

    // 補正処理を実行
    correctDistortion(cloud_msg, output);
  }
}
```

**利点:**
- メインスレッドはデータ受信に専念（低レイテンシ）
- 処理スレッドでCPU負荷の高い補正処理を実行
- データが揃った時点で即座に処理開始

**English:**

This package adopts Livox SDK's point cloud correction data synchronization mechanism:

```cpp
// Callback (main thread)
void pointCloudCallback(const PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_buffer_);
  cloud_buffer_.push_back(msg);

  // Notify processing thread (Livox-style)
  sig_buffer_.notify_all();
}

// Processing thread (background)
void processingLoop() {
  while (!shutdown_) {
    std::unique_lock<std::mutex> lock(mtx_buffer_);

    // Wait until data is ready
    sig_buffer_.wait(lock, [this] {
      return shutdown_ || !cloud_buffer_.empty();
    });

    // Get data
    auto cloud_msg = cloud_buffer_.front();
    cloud_buffer_.pop_front();

    // Unlock before processing (Livox-style)
    lock.unlock();

    // Execute correction
    correctDistortion(cloud_msg, output);
  }
}
```

**Benefits:**
- Main thread focuses on data reception (low latency)
- Processing thread handles CPU-intensive correction
- Processes immediately when data is ready

---

## トラブルシューティング / Troubleshooting

### 補正が動作しない / Correction Not Working

```bash
# データが届いているか確認
ros2 topic echo /imu/data
ros2 topic echo /odom
ros2 topic echo /velodyne_points

# ノードの状態確認
ros2 node info /distortion_corrector

# パラメータ確認
ros2 param list /distortion_corrector
```

### IMU/Odomデータが不足している / Insufficient IMU/Odom Data

ログに "Not enough IMU/Odom data" と表示される場合：
- IMU/Odomの周波数を確認（50Hz以上推奨）
- `max_buffer_size` を増やす
- `scan_duration` を調整

---

## 参考 / References

- [Livox Cloud Undistortion](https://github.com/Livox-SDK/livox_cloud_undistortion)
- [Autoware Universe - Distortion Corrector](https://github.com/autowarefoundation/autoware_universe)

---

## ライセンス / License

Apache License 2.0

---

## 開発者 / Developer

Created with Claude Code
