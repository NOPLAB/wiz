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

### オプション2: DockerでROS2連携（フルROS2統合）

Dockerを使用して、サンプルパブリッシャーを含む完全なROS2環境を実行します。

1. **Dockerサービスを起動:**
   ```bash
   ./scripts/run-ros2-example.sh
   ```

   以下のサービスが起動します:
   - `ros2-example`: サンプルセンサーデータをパブリッシュするROS2ノード
   - `wiz-server-humble`: ROS2ブリッジ付きのwizサーバー

2. **フロントエンドをローカルで起動:**
   ```bash
   cargo run -p wiz-frontend
   ```

3. **接続して可視化:**
   - `ws://localhost:9090/ws` に接続
   - `/scan` または `/velodyne_points` をサブスクライブ

### オプション3: 自分のROS2システムに接続

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
| `/scan_front` | LaserScan | 前方向けレーザースキャン |

### ROS2サンプルトピック

Docker ROS2サンプル（`ros2-example`）使用時:

| トピック | 型 | 説明 |
|----------|------|------|
| `/velodyne_points` | PointCloud2 | スパイラル点群 |
| `/scan` | LaserScan | 障害物付き360度スキャン |

## ファイル構成

- `ros2_publisher.py` - サンプルセンサーデータをパブリッシュするPython ROS2ノード
- `../scripts/run-demo.sh` - モックデータでサーバーを実行
- `../scripts/run-ros2-example.sh` - Docker ROS2サンプルを実行
- `../scripts/run-server.sh` - サーバーのみ実行
- `../scripts/run-frontend.sh` - フロントエンドのみ実行

## Dockerコマンド

イメージをビルド:
```bash
docker-compose build
```

ROS2サンプルを実行:
```bash
docker-compose --profile example up ros2-example wiz-server-humble
```

wiz-serverのみを実行（外部ROS2使用時）:
```bash
docker-compose up wiz-server-humble
```

## トラブルシューティング

### 接続拒否
- サーバーがポート9090で動作していることを確認
- 他のプロセスがポートを使用していないか確認: `lsof -i :9090`

### データが受信されない
- Topicsパネルでトピックをサブスクライブしたことを確認
- サーバーログでサブスクリプション確認メッセージを確認

### Dockerネットワークの問題
- ROS2 DDS通信には `--net=host` を使用
- 必要に応じて `ROS_DOMAIN_ID` 環境変数を設定
