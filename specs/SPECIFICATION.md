# wiz - ROS2 Visualization Tool Specification

## Overview

wizはROS2のRviz代替となる次世代可視化ツールです。Rustによる型安全なレンダリングエンジンとWebGPUを活用し、ブラウザとネイティブアプリケーションの両方で同等に動作します。

### Target ROS2 Versions
- **ROS2 Humble** (Ubuntu 22.04, LTS 2027)
- **ROS2 Jazzy** (Ubuntu 24.04, LTS)

### Design Principles
- **性能優先**: 大規模点群処理、GPU instancing最適化
- **型安全**: Rust + cxxによる安全なFFI
- **クロスプラットフォーム**: Web/Native同等サポート

## Architecture

```
┌───────────────────────────────────────────────────────────────────────┐
│                           wiz System                                  │
├───────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌──────────────────┐                ┌─────────────────────────────┐  │
│  │     Frontend     │   WebSocket    │     Backend (単一プロセス)    │  │
│  │    (Renderer)    │◄──────────────►│                             │  │
│  │                  │                │  ┌─────────────────────────┐│  │
│  │  ┌────────────┐  │                │  │    Rust Server Layer    ││  │
│  │  │   WebGPU   │  │                │  │  (axum + tokio)         ││  │
│  │  │   Engine   │  │                │  └───────────┬─────────────┘│  │
│  │  │            │  │                │              │ cxx FFI      │  │
│  │  │ - Points   │  │                │  ┌───────────▼─────────────┐│  │
│  │  │ - Lines    │  │                │  │   ROS2 Bridge (C++)     ││  │
│  │  │ - Meshes   │  │                │  │   rclcpp linked         ││  │
│  │  └────────────┘  │                │  └───────────┬─────────────┘│  │
│  │                  │                │              │              │  │
│  │  Rust/WASM       │                │              ▼              │  │
│  │  or Native       │                │       ┌───────────┐         │  │
│  └──────────────────┘                │       │   ROS2    │         │  │
│                                      │       │  Network  │         │  │
│                                      │       └───────────┘         │  │
│                                      └─────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────────┘
```

### Single Process Design
ROS2 BridgeとBackend Serverは同一プロセスで動作し、cxx経由で直接連携します。
- **利点**: レイテンシ最小化、デプロイ簡素化
- **構成**: Rustバイナリにrclcpp (C++) を静的/動的リンク

## Components

### 1. Frontend (Renderer) - Rust

レンダリングエンジンはRustで実装し、WebGPUを使用してGPUアクセラレーションを実現します。

#### 技術スタック
- **言語**: Rust
- **Graphics API**: wgpu (WebGPU abstraction layer)
- **GUI Framework**: egui + eframe + egui_dock
- **WebSocket**: ewebsock (クロスプラットフォーム)
- **Web Target**: wasm32-unknown-unknown (WASM)
- **Native Target**: x86_64-unknown-linux-gnu, etc.

#### 主要機能
- 3D/2Dシーンレンダリング
- カメラ制御（パン、ズーム、回転）- オービットカメラ
- 複数のビジュアライゼーションプラグイン
- リアルタイムデータストリーミング表示
- Dock型パネルレイアウト

#### カメラ設定
- **タイプ**: オービットカメラ
- **FOV**: 60° (設定可能)
- **Near/Far**: 0.1 / 1000.0
- **デフォルト位置**: (5, 5, 5)
- **上方向**: Z軸
- **ズーム範囲**: 0.1 ～ 500.0

#### 性能最適化戦略
- **GPU Instancing**: 大量の同一プリミティブ（点、マーカー）を1回のドローコールで描画
- **Frustum Culling**: 視錐台外のオブジェクトをスキップ
- **LOD (Level of Detail)**: 距離に応じて点群密度を動的調整
- **Double Buffering**: データ更新とレンダリングの並行処理
- **Compute Shader**: 点群のソート・フィルタリングをGPUで実行

### 2. Backend Server - Rust

WebSocketサーバーとしてフロントエンドとROS2ブリッジ間の通信を管理します。

#### 技術スタック
- **言語**: Rust
- **WebSocket**: axum (tower-http)
- **Serialization**: serde + rmp-serde (MessagePack)
- **Async Runtime**: tokio

#### 主要機能
- WebSocket接続管理 (`/ws`)
- ヘルスチェックエンドポイント (`/health`)
- メッセージルーティング
- トピックサブスクリプション管理
- データ変換・フィルタリング

#### モックデータ生成
ROS2環境がない場合でも動作確認できるよう、モックデータを10Hzで生成:
- **LaserScan**: 360レイ、動的障害物付き
- **PointCloud2**: 5000点のアニメーションスパイラル
- **Ground Plane**: 50x50グリッド (波アニメーション)

### 3. ROS2 Bridge - C++

ROS2ネットワークとの通信を担当するC++コンポーネント。Rustプロセス内にリンクされます。

#### 技術スタック
- **言語**: C++17/20
- **ROS2 Client**: rclcpp
- **FFI**: cxx (型安全な双方向バインディング)

#### 主要機能
- ROS2トピックのSubscribe
- TF2変換ルックアップ

#### cxx Bridge Example
```rust
// Rust側
#[cxx::bridge]
mod ffi {
    extern "C++" {
        include!("wiz_ros2_bridge/bridge.hpp");

        type Ros2Bridge;
        fn create_bridge(namespace: &str) -> UniquePtr<Ros2Bridge>;
        fn subscribe_pointcloud2(
            self: &Ros2Bridge,
            topic: &str,
            callback: fn(data: &[u8], timestamp: f64),
        );
        fn lookup_transform(
            self: &Ros2Bridge,
            target: &str,
            source: &str,
        ) -> Result<Transform>;
    }
}
```

## Data Flow

```
ROS2 Topic ──► ROS2 Bridge ──► Backend Server ──► WebSocket ──► Frontend
                 (C++)           (Rust)                         (Rust)
                   │                │                              │
                   ▼                ▼                              ▼
              Deserialize      Serialize to           Deserialize &
              ROS2 Message     Binary Format          Render
```

## Supported Message Types

### Core (MVP)
| Message Type | Description | Priority |
|-------------|-------------|----------|
| sensor_msgs/PointCloud2 | 3D Point Cloud | High |
| sensor_msgs/LaserScan | 2D Laser Scan | High |
| tf2_msgs/TFMessage | Transform Tree | High |
| geometry_msgs/PoseStamped | Robot Pose | Medium |
| nav_msgs/Path | Navigation Path | Medium |
| visualization_msgs/Marker | Custom Markers | Medium |
| visualization_msgs/MarkerArray | Marker Arrays | Medium |

### Extended
| Message Type | Description | Priority |
|-------------|-------------|----------|
| sensor_msgs/Image | Camera Image | Medium |
| nav_msgs/OccupancyGrid | 2D Map | Low |
| nav_msgs/Odometry | Odometry | Low |

### Future
| Message Type | Description |
|-------------|-------------|
| URDF/SDF | Robot Model |
| octomap_msgs/Octomap | 3D Occupancy |

## WebSocket Protocol

### Connection
```
ws://[host]:[port]/ws
```

### Message Format

すべてのメッセージはMessagePack形式でシリアライズされます。

```rust
// Rust型定義
#[derive(Serialize, Deserialize)]
#[serde(tag = "type")]
enum ClientMessage {
    #[serde(rename = "subscribe")]
    Subscribe {
        id: String,
        topic: String,
        msg_type: String,
        #[serde(default)]
        throttle_rate: Option<u32>,  // Hz
    },
    #[serde(rename = "unsubscribe")]
    Unsubscribe { id: String },
    #[serde(rename = "tf_lookup")]
    TfLookup {
        target_frame: String,
        source_frame: String,
    },
    #[serde(rename = "list_topics")]
    ListTopics,
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "type")]
enum ServerMessage {
    #[serde(rename = "data")]
    Data {
        topic: String,
        msg_type: String,
        timestamp: f64,
        #[serde(with = "serde_bytes")]
        payload: Vec<u8>,
    },
    #[serde(rename = "tf")]
    Transform {
        target_frame: String,
        source_frame: String,
        transform: Transform3D,
    },
    #[serde(rename = "topics")]
    Topics {
        topics: Vec<TopicInfo>,
    },
    #[serde(rename = "subscribed")]
    Subscribed {
        id: String,
        topic: String,
    },
    #[serde(rename = "unsubscribed")]
    Unsubscribed {
        id: String,
    },
    #[serde(rename = "error")]
    Error { message: String },
}
```

## Project Structure

```
wiz/
├── Cargo.toml                 # Workspace root
├── crates/
│   ├── wiz-core/              # 共通型・トレイト定義
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── messages.rs    # ROS2メッセージ対応の内部型
│   │       └── transform.rs   # 変換行列
│   │
│   ├── wiz-renderer/          # WebGPUレンダリングエンジン
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── renderer.rs    # メインレンダラー
│   │       ├── camera.rs      # オービットカメラ制御
│   │       ├── pipeline.rs    # レンダリングパイプラインビルダー
│   │       ├── grid.rs        # グリッドレンダラー
│   │       ├── point_cloud.rs # 点群レンダラー
│   │       ├── laser_scan.rs  # LaserScanレンダラー
│   │       ├── tf_axis.rs     # TF座標軸レンダラー
│   │       └── shaders/       # WGSLシェーダー
│   │           ├── grid.wgsl
│   │           ├── point_cloud.wgsl
│   │           ├── laser_scan.wgsl
│   │           └── tf_axis.wgsl
│   │
│   ├── wiz-frontend/          # GUIアプリケーション (egui)
│   │   └── src/
│   │       ├── main.rs        # Nativeエントリーポイント
│   │       ├── lib.rs         # WASMエントリーポイント
│   │       ├── app.rs         # メインアプリケーション
│   │       ├── app_state.rs   # アプリケーション状態管理
│   │       ├── viewport_state.rs  # 3Dビューポート状態
│   │       ├── ws_client.rs   # WebSocketクライアント
│   │       └── panels/        # UIパネル
│   │           ├── mod.rs
│   │           ├── viewport.rs    # 3Dビューポート
│   │           ├── topics.rs      # トピック選択
│   │           ├── displays.rs    # 表示設定
│   │           ├── tf_tree.rs     # TFツリー
│   │           └── performance.rs # パフォーマンスモニター
│   │
│   ├── wiz-server/            # WebSocketサーバー + ROS2ブリッジ
│   │   ├── src/
│   │   │   ├── main.rs        # サーバーエントリー
│   │   │   ├── ws.rs          # WebSocket処理
│   │   │   ├── state.rs       # サーバー状態管理
│   │   │   ├── mock_data.rs   # モックデータ生成
│   │   │   ├── bridge.rs      # cxx FFIラッパー
│   │   │   └── ffi.rs         # cxx FFI定義
│   │   ├── cxx/               # C++ ROS2ブリッジ
│   │   │   ├── bridge.hpp
│   │   │   └── bridge.cpp
│   │   ├── build.rs           # cxx ビルド設定
│   │   └── CMakeLists.txt     # colcon統合用
│   │
│   └── wiz-protocol/          # 通信プロトコル定義
│       └── src/
│           ├── lib.rs
│           └── codec.rs       # MessagePackエンコード/デコード
│
├── web/                       # Web用アセット
│   ├── index.html
│   └── pkg/                   # wasm-bindgen出力
│
├── config/                    # 設定ファイル例
│   └── default.toml
│
└── docker/
    ├── Dockerfile.server      # サーバービルド用
    └── Dockerfile.humble      # ROS2 Humble環境
```

## Build System

### Prerequisites
```bash
# ROS2 Humble の場合
source /opt/ros/humble/setup.bash

# ROS2 Jazzy の場合
source /opt/ros/jazzy/setup.bash

# Rust ツールチェーン
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli trunk
```

### Server Build (Native + ROS2 Bridge)
```bash
# ROS2環境をsourceした状態で実行
# build.rs が自動的にcxxとrclcppをリンク
cargo build --release -p wiz-server
```

### Frontend Build

#### Native
```bash
cargo build --release -p wiz-frontend
```

#### WASM (Web)
```bash
# trunk を使用した開発サーバー
cd crates/wiz-frontend
trunk serve

# プロダクションビルド
trunk build --release
```

### Docker Build
```bash
# Humble環境
docker build -f docker/Dockerfile.humble -t wiz-server:humble .

# Jazzy環境
docker build -f docker/Dockerfile.jazzy -t wiz-server:jazzy .
```

## Configuration

すべての設定ファイルはTOML形式です。

### Server Config (`config/server.toml`)
```toml
[server]
host = "0.0.0.0"
port = 9090
max_connections = 100

[ros2]
namespace = ""
use_sim_time = false

[performance]
# 点群処理
max_point_cloud_points = 2_000_000
point_cloud_lod_enabled = true

# スロットリング
default_throttle_hz = 30
```

### Display Config (`config/display.toml`)
```toml
[camera]
fov = 60.0
near = 0.1
far = 1000.0

[grid]
enabled = true
size = 10.0
divisions = 10

[[displays]]
type = "PointCloud2"
topic = "/velodyne_points"
color_mode = "intensity"  # "flat", "intensity", "rgb", "height"
point_size = 2.0
visible = true

[[displays]]
type = "LaserScan"
topic = "/scan"
color = [1.0, 0.0, 0.0, 1.0]  # RGBA
line_width = 1.0

[[displays]]
type = "TF"
enabled = true
show_names = true
axis_length = 0.5
```

## Performance Targets

| Metric | Target |
|--------|--------|
| Point Cloud (1M points) | 30+ FPS |
| Image (1080p) | 30+ FPS |
| Latency (end-to-end) | < 50ms |
| Memory (typical scene) | < 500MB |
| WASM bundle size | < 10MB |

## Security Considerations

- WebSocket接続は認証トークンをサポート
- ローカルホスト以外からの接続はデフォルトで無効
- TLS/SSL対応（オプション）

## Development Phases

### Phase 1: Foundation (MVP) ✅ 完了
プロジェクト基盤とコア機能の実装

- [x] **プロジェクト構造**
  - [x] Cargo workspaceセットアップ
  - [x] cxx FFI基本構成
  - [x] CI/CD (GitHub Actions)

- [x] **ROS2 Bridge (C++)** ※モック実装、ros2 feature有効時に実ROS2連携
  - [x] rclcpp初期化/終了
  - [x] PointCloud2 Subscriber
  - [x] LaserScan Subscriber
  - [x] TF2 lookup
  - [x] トピック一覧取得

- [x] **WebSocket Server (Rust)**
  - [x] axum WebSocketエンドポイント (`/ws`)
  - [x] ヘルスチェックエンドポイント (`/health`)
  - [x] MessagePackシリアライズ
  - [x] トピック一覧API
  - [x] サブスクリプション管理
  - [x] モックデータ生成 (10Hz)
    - [x] LaserScan (360レイ、動的障害物)
    - [x] PointCloud2 (5000点スパイラル)
    - [x] Ground Plane (50x50グリッド)

- [x] **Renderer (Rust/WebGPU)**
  - [x] wgpu初期化 (Native + WASM)
  - [x] オービットカメラ制御 (回転、パン、ズーム)
  - [x] グリッド描画
  - [x] 点群レンダリング (可変サイズ、アルファ)
  - [x] LaserScan描画 (ライン + ポイント)
  - [x] TF座標軸描画 (最大256フレーム、インスタンシング)
  - [x] WGSLシェーダー一式

- [x] **Frontend (egui)**
  - [x] 接続設定UI
  - [x] トピック選択パネル (フィルタ機能付き)
  - [x] 表示設定パネル (Display管理)
  - [x] TFツリーパネル
  - [x] パフォーマンスパネル (FPS、メモリ、帯域幅)
  - [x] 3Dビューポート統合 (egui_wgpu)
  - [x] Dock型レイアウト (egui_dock)
  - [x] WASMビルドサポート

- [x] **WebSocketクライアント**
  - [x] クロスプラットフォーム対応 (ewebsock)
  - [x] 接続状態管理
  - [x] イベントキュー処理

### Phase 2: Core Features 🔄 進行中
可視化機能の拡充

- [x] TFフレーム表示 (座標軸)
- [x] PoseStamped表示 (矢印による姿勢可視化)
- [x] Marker/MarkerArray (Cube, Sphere, Cylinder, Arrow, LineStrip, LineList, CubeList, SphereList, Points)
- [ ] Path表示 ※メッセージ型定義済み
- [ ] 表示設定の永続化

### Phase 3: Extended
追加メッセージタイプと機能

- [ ] Image表示 (テクスチャ)
- [ ] OccupancyGrid
- [x] パフォーマンスパネル ※基本実装完了
- [ ] 複数ビューポート

### Phase 4: Polish
品質向上

- [ ] URDF表示
- [ ] ドキュメント
- [x] E2Eテスト (wiz-server WebSocket/HTTP)

## Dependencies

### Rust (Cargo.toml)
```toml
[workspace]
resolver = "2"
members = [
    "crates/wiz-core",
    "crates/wiz-renderer",
    "crates/wiz-frontend",
    "crates/wiz-server",
    "crates/wiz-protocol",
]

[workspace.dependencies]
# Graphics
wgpu = "23"
egui = "0.30"
eframe = "0.30"
egui_dock = "0.15"
egui_wgpu = "0.30"

# Async
tokio = { version = "1", features = ["full"] }
axum = "0.8"

# Serialization
serde = { version = "1", features = ["derive"] }
serde_bytes = "0.11"
rmp-serde = "1"
toml = "0.8"

# FFI
cxx = "1"
cxx-build = "1"

# Math
glam = "0.29"

# Utilities
tracing = "0.1"
tracing-subscriber = "0.3"
thiserror = "2"
uuid = { version = "1", features = ["v4"] }
bytemuck = { version = "1", features = ["derive"] }

# WebSocket (cross-platform)
ewebsock = "0.7"
```

### C++ (ROS2 Bridge)
CMakeLists.txtで指定:
```cmake
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
```

## License

MIT
