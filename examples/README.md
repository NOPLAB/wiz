# wiz サンプル

このディレクトリには、wizをテストするためのサンプルとスクリプトが含まれています。

## クイックスタート

### オプション1: モックデータ（ROS2不要）

サーバーにはシミュレートされたセンサーデータを生成するモックデータジェネレーターが組み込まれています。

1. **サーバーを起動:**
   ```bash
   ./scripts/run-demo.sh
   ```
   または手動で:
   ```bash
   cargo run -p wiz-server
   ```

2. **フロントエンドを起動（別のターミナルで）:**
   ```bash
   cargo run -p wiz-frontend
   ```

3. **接続してサブスクライブ:**
   - フロントエンドに接続ダイアログが表示されます
   - `ws://localhost:9090/ws` に接続
   - Topicsパネルでトピックをサブスクライブ
   - 3Dビューポートでデータが可視化されます

### オプション2: TurtleBot3 Gazeboシミュレーション（推奨）

Dockerを使用して、TurtleBot3ロボットのGazeboシミュレーションを実行します。
RVizと同じデータを可視化できます。

1. **X11許可を設定（Linux/WSL2）:**
   ```bash
   xhost +local:docker
   ```

2. **Dockerサービスを起動:**
   ```bash
   ./scripts/run-ros2-gazebo-example.sh
   ```

   以下のサービスが起動します:
   - `gazebo-turtlebot3`: TurtleBot3 (waffle_pi) のGazeboシミュレーション
   - `wiz-server-humble`: ROS2ブリッジ付きのwizサーバー

3. **フロントエンドをローカルで起動:**
   ```bash
   cargo run -p wiz-frontend
   ```

4. **接続して可視化:**
   - `ws://localhost:9090/ws` に接続
   - `/scan`、`/camera/depth/points` などをサブスクライブ

5. **ロボットを操作（オプション）:**
   ```bash
   docker exec -it $(docker ps -qf name=gazebo) ros2 run turtlebot3_teleop teleop_keyboard
   ```

### オプション3: ROS2パッケージとして使用（推奨）

wizをROS2パッケージとしてビルドし、launchファイルから起動できます。

1. **パッケージをビルド:**
   ```bash
   cd /path/to/wiz
   colcon build --packages-select wiz
   source install/setup.bash
   ```

2. **wizを起動（サーバー + フロントエンド）:**
   ```bash
   ros2 launch wiz wiz.launch.py
   ```

3. **サーバーのみ起動:**
   ```bash
   ros2 launch wiz server.launch.py port:=9090
   ```

4. **他のROS2パッケージから呼び出し:**
   ```python
   # your_robot.launch.py
   IncludeLaunchDescription(
       PythonLaunchDescriptionSource([
           FindPackageShare('wiz'), '/launch/wiz.launch.py'
       ])
   )
   ```

### オプション4: wiz_exampleデモ（Docker、軽量）

Gazeboなしで軽量なデモを実行したい場合:

1. **Dockerサービスを起動:**
   ```bash
   ./scripts/run-ros2-example.sh
   ```

   以下のサービスが起動します:
   - `wiz-example`: デモノード（MarkerArray、PointCloud2、TF）
   - `wiz-server-humble`: ROS2ブリッジ付きのwizサーバー

2. **フロントエンドをローカルで起動:**
   ```bash
   cargo run -p wiz-frontend
   ```

3. **接続して可視化:**
   - `ws://localhost:9090/ws` に接続
   - `/demo_markers`、`/demo_pointcloud` をサブスクライブ

### オプション5: 自分のROS2システムに接続（手動ビルド）

システム上でROS2が動作している場合:

1. **ROS2サポート付きでwiz-serverをビルド:**
   ```bash
   source /opt/ros/humble/setup.bash
   cargo build --release -p wiz-server --features ros2
   ```

2. **サーバーを実行:**
   ```bash
   ./target/release/wiz-server
   ```

3. **フロントエンドを起動して接続**

## 利用可能なトピック

### モックデータトピック

| トピック | 型 | 説明 |
|----------|------|------|
| `/velodyne_points` | PointCloud2 | アニメーション付き3Dスパイラル点群 |
| `/ground_plane` | PointCloud2 | チェッカーボード地面 |
| `/scan` | LaserScan | 動く障害物を含む360度レーザースキャン |
| `/robot_pose` | PoseStamped | 8の字パターンで移動するロボット姿勢 |

### TurtleBot3 Gazeboトピック

TurtleBot3 Gazeboシミュレーション使用時:

| トピック | 型 | 説明 |
|----------|------|------|
| `/scan` | LaserScan | LiDARスキャン（360度） |
| `/odom` | Odometry | オドメトリ |
| `/tf` | TFMessage | 座標変換ツリー |
| `/camera/image_raw` | Image | RGBカメラ画像 |
| `/camera/depth/image_raw` | Image | 深度画像 |
| `/camera/depth/points` | PointCloud2 | 深度点群 |
| `/imu` | Imu | IMUデータ |

### wiz_exampleデモトピック

wiz_exampleパッケージのデモノードを使用する場合:

```bash
# wiz_exampleもビルド
colcon build --packages-select wiz wiz_example
source install/setup.bash

# デモを実行（wiz + デモノード）
ros2 launch wiz_example demo_with_wiz.launch.py
```

| トピック | 型 | 説明 |
|----------|------|------|
| `/demo_markers` | MarkerArray | アニメーション付き形状（キューブ、球、シリンダー、矢印） |
| `/demo_pointcloud` | PointCloud2 | 虹色のスパイラル点群 |

TFフレーム:
```
map → odom → base_link (8の字パターンで移動)
              ├── laser_frame
              ├── camera_frame
              └── arm_base → arm_link1 → arm_link2 → end_effector
```

## ファイル構成

### ROS2パッケージ
- `../ros2/wiz/` - wizメインパッケージ（launchファイル、Rustビルド統合）
- `ros2/wiz_example/` - デモパッケージ（MarkerArray、PointCloud2、TFのデモ）

### スクリプト・Docker
- `../docker/Dockerfile.gazebo` - TurtleBot3 Gazebo環境
- `../docker/Dockerfile.wiz-example` - wiz_exampleデモ環境
- `../scripts/run-demo.sh` - モックデータでサーバーを実行
- `../scripts/run-ros2-example.sh` - wiz_exampleデモを実行（Docker使用、軽量）
- `../scripts/run-ros2-gazebo-example.sh` - TurtleBot3 Gazeboシミュレーションを実行（Docker使用）
- `../scripts/run-server.sh` - サーバーのみ実行
- `../scripts/run-frontend.sh` - フロントエンドのみ実行

## Dockerコマンド

イメージをビルド:
```bash
docker compose build
```

TurtleBot3 Gazeboを実行:
```bash
docker compose --profile example up gazebo-turtlebot3 wiz-server-humble
```

wiz-serverのみを実行（外部ROS2使用時）:
```bash
docker compose up wiz-server-humble
```

## トラブルシューティング

### 接続拒否
- サーバーがポート9090で動作していることを確認
- 他のプロセスがポートを使用していないか確認: `lsof -i :9090`

### データが受信されない
- Topicsパネルでトピックをサブスクライブしたことを確認
- サーバーログでサブスクリプション確認メッセージを確認

### Gazebo GUIが表示されない
- `xhost +local:docker` を実行したか確認
- `DISPLAY` 環境変数が正しく設定されているか確認
- WSL2の場合、WSLgがインストールされているか確認

### Dockerネットワークの問題
- ROS2 DDS通信には `--net=host` を使用
- 必要に応じて `ROS_DOMAIN_ID` 環境変数を設定
