# wiz UI/UX Specification

## Design Principles

- **Dock型レイアウト**: パネルを自由に配置・リサイズ可能
- **Web/Native完全同一**: 見た目・操作すべて同じ体験
- **ダーク/ライト両対応**: ユーザー設定で切り替え

## Layout Structure ✅ 実装済み (egui_dock)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              wiz                                        │
├───────────────┬─────────────────────────────────────┬───────────────────┤
│               │                                     │                   │
│   Topics      │                                     │    Displays       │
│   Panel       │                                     │    Panel          │
│               │          3D Viewport                │                   │
│   ┌─────────┐ │                                     │   ┌─────────────┐ │
│   │ /scan   │ │              ┌───┐                  │   │PointCloud2  │ │
│   │ /points │ │              │ Z │                  │   │ └─ /points  │ │
│   │ /tf     │ │            ──┼───┼── X              │   │ LaserScan   │ │
│   │ ...     │ │              │   │                  │   │ └─ /scan    │ │
│   └─────────┘ │              Y                      │   └─────────────┘ │
│               │                                     │                   │
├───────────────┴─────────────────────────────────────┼───────────────────┤
│   TF Tree Panel                                     │  Performance      │
│   map ─┬─ odom ─── base_link ─┬─ laser_frame       │  FPS: 60          │
│        └─ camera_link         └─ wheel_left        │  Points: 1.2M     │
│                                                     │  Latency: 12ms    │
└─────────────────────────────────────────────────────┴───────────────────┘
```

**注意**: Menu BarとToolbarは未実装。タイトルバーのみ表示。

### Default Panel Layout
| 位置 | パネル | サイズ |
|------|--------|--------|
| 左 | Topics | 200px |
| 中央 | 3D Viewport | flex |
| 右 | Displays | 280px |
| 下左 | TF Tree | 50% |
| 下右 | Performance | 50% |

### Docking Behavior
- パネルはドラッグで移動、エッジにドッキング可能
- タブ化で同一領域に複数パネル配置
- ダブルクリックでフロート/ドック切り替え
- パネル閉じる→メニューから再表示

## Color Theme

### Dark Theme (Default)
```
Background:
  Primary:    #1a1a1a
  Secondary:  #252525
  Panel:      #2d2d2d

Foreground:
  Primary:    #e0e0e0
  Secondary:  #a0a0a0
  Disabled:   #606060

Accent:
  Primary:    #4a9eff  (Blue)
  Success:    #4ade80  (Green)
  Warning:    #fbbf24  (Yellow)
  Error:      #f87171  (Red)

3D Viewport:
  Background: #1e1e1e
  Grid:       #3a3a3a
  Axes:       X:#ff4444 Y:#44ff44 Z:#4444ff
```

### Light Theme
```
Background:
  Primary:    #f5f5f5
  Secondary:  #e8e8e8
  Panel:      #ffffff

Foreground:
  Primary:    #1a1a1a
  Secondary:  #606060
  Disabled:   #a0a0a0

3D Viewport:
  Background: #e0e0e0
  Grid:       #c0c0c0
```

## Panels

### 1. Topics Panel ✅ 実装済み

利用可能なROS2トピックを一覧表示し、表示に追加する。

```
┌─ Topics ──────────────────────[×]─┐
│ [🔍 Filter...]                    │
│                                   │
│   /velodyne_points  PointCloud2   │
│   /scan             LaserScan     │
│   /tf               TFMessage     │
│   /robot_pose       PoseStamped   │
│                                   │
│ [↻ Refresh]                       │
└───────────────────────────────────┘
```

#### 実装済み機能 ✅
- フィルタ検索 (大文字小文字区別なし)
- トピック一覧表示 (トピック名 + メッセージ型)
- ダブルクリックでDisplayに追加
- Refreshボタンでトピック再取得
- 選択ハイライト

#### 未実装 ❌
- メッセージ型によるグルーピング
- 右クリックコンテキストメニュー
- ドラッグ&ドロップ

### 2. Displays Panel ✅ 実装済み

アクティブな表示項目の設定を管理する。

```
┌─ Displays ────────────────────[×]─┐
│                                   │
│ ▼ ☑ PointCloud2: /velodyne_points │
│   │  Topic: /velodyne_points      │
│   │  Color: [Intensity ▼]         │
│   │  Size:  [===●===] 2.0         │
│   │  Alpha: [======●=] 0.8        │
│   │  [Remove]                     │
│   │                               │
│ ▼ ☑ LaserScan: /scan              │
│   │  Color: [■] #ff4444           │
│   │  Width: [===●===] 1.0         │
│   │                               │
│ ▶ ☐ TF Frames                     │
│                                   │
│ [+ Add Display ▼]                 │
└───────────────────────────────────┘

☑ = Visible
☐ = Hidden
▼ = Expanded
▶ = Collapsed
```

#### Display Types & Settings

| Type | Settings | 実装状況 |
|------|----------|---------|
| PointCloud2 | point_size, alpha, color | ✅ 実装済み |
| LaserScan | color, line_width, alpha, show_points | ✅ 実装済み |
| TF | axis_length, show_all_frames | ✅ 実装済み |
| Pose | color, arrow_length, arrow_width | ✅ 実装済み |
| Marker | (auto from message) | ❌ 未実装 |
| Path | color, line_width | ❌ 未実装 |

#### 現在のデフォルト設定
```rust
DisplaySettings {
    point_size: 2.0,
    alpha: 1.0,
    color: [1.0, 0.0, 0.0, 1.0],  // Red
}
```

### 3. TF Tree Panel ✅ 実装済み

TFフレームの階層構造をツリー表示する。

```
┌─ TF Tree ─────────────────────[×]─┐
│ [🔍 Filter...]  Fixed: [map ▼]    │
│                                   │
│ map                               │
│ └─┬─ odom                         │
│   │  └── base_link                │
│   │      ├── laser_frame          │
│   │      ├── camera_link          │
│   │      └─┬─ wheel_left          │
│   │        └── wheel_right        │
│   └── gps_frame                   │
│                                   │
│ Selected: base_link               │
│ Parent: odom                      │
│ Position: [1.23, 0.45, 0.00]      │
│ Rotation: [0.0, 0.0, 45.0]°       │
└───────────────────────────────────┘
```

#### 操作
- クリック: フレーム選択、詳細表示
- ダブルクリック: カメラをそのフレームにフォーカス
- 右クリック: Fixed Frameに設定

### 4. Performance Panel ✅ 実装済み

リアルタイムのパフォーマンス指標を表示。

```
┌─ Performance ─────────────────[×]─┐
│                                   │
│  Rendering                        │
│  ├─ FPS:        60.0              │
│  ├─ Frame Time: 16.2 ms           │
│                                   │
│  Data                             │
│  ├─ Points:     1,234,567         │
│  ├─ Triangles:  45,678            │
│                                   │
│  Network                          │
│  ├─ Latency:    12 ms             │
│  ├─ Bandwidth:  24.5 MB/s         │
│  └─ Messages:   342/s             │
│                                   │
│  Memory                           │
│  ├─ GPU:        256 MB            │
│  └─ CPU:        128 MB            │
│                                   │
│  [FPS History Graph ▂▃▅▇▅▃▂▃▅▇] │
└───────────────────────────────────┘
```

#### 実装詳細
- **FPS履歴**: 120サンプル保持
- **フレームタイム履歴**: 60サンプル保持
- **リアルタイム更新**: 毎フレーム

## 3D Viewport ✅ 実装済み

### Camera Control (Orbit Style)

| 操作 | アクション | 実装状況 |
|------|----------|---------|
| 中ボタンドラッグ | 回転 (Orbit) | ✅ |
| Shift + 中ボタンドラッグ | パン (Pan) | ✅ |
| ホイールスクロール | ズーム (Dolly) | ✅ |
| Ctrl + 中ボタンドラッグ | ズーム (Dolly) | ✅ |
| テンキー 1/3/7 | Front/Right/Top ビュー | ❌ |
| テンキー 5 | 透視/正投影 切り替え | ❌ |
| Home / テンキー . | 全体表示 (Fit All) | ❌ |
| F | 選択オブジェクトにフォーカス | ❌ |

#### カメラパラメータ
- **デフォルト位置**: (5, 5, 5)
- **上方向ベクトル**: Z軸
- **FOV**: 60°
- **Near/Far**: 0.1 / 1000.0
- **ズーム範囲**: 0.1 ～ 500.0

### Viewport Overlay

```
┌─────────────────────────────────────────────────┐
│ Frame: map                              [⚙]    │  ← 左上: 情報
│                                                │
│                                                │
│                     ↑Z                         │
│                     │                          │
│                   ──┼──→ X                     │  ← 右下: 座標軸
│                    ╱                           │
│                   Y                            │
│                                                │
│                                  [Persp] [🔲]  │  ← 右下: ビューモード
└─────────────────────────────────────────────────┘
```

### Selection
- 左クリック: オブジェクト/ポイント選択
- 選択時: アウトライン表示 + プロパティパネル表示

## Toolbar ❌ 未実装

将来実装予定:
```
┌─────────────────────────────────────────────────────────────────────────┐
│ [💾 Save] [📂 Load] │ [🔌 Connect] ws://localhost:9090 [●]             │
│                     │                                                   │
│ Frame: [map ▼]      │ [Grid ☑] [TF Axes ☑] [Origin ☑]                  │
└─────────────────────────────────────────────────────────────────────────┘

● Green  = Connected
● Yellow = Connecting
● Red    = Disconnected
```

## Menu Bar ❌ 未実装

将来実装予定:
```
File    Edit    View    Displays    Help
```

### File Menu
- New Layout
- Open Config... (Ctrl+O)
- Save Config (Ctrl+S)
- Save Config As...
- ──────────
- Exit

### Edit Menu
- Preferences...

### View Menu
- Topics Panel
- Displays Panel
- TF Tree Panel
- Performance Panel
- ──────────
- Reset Layout
- ──────────
- Dark Theme / Light Theme

### Displays Menu
- Add PointCloud2
- Add LaserScan
- Add TF
- Add Marker
- Add Path
- Add Pose
- ──────────
- Remove All

### Help Menu
- Keyboard Shortcuts
- About wiz

## Keyboard Shortcuts ❌ 未実装

将来実装予定:

| Key | Action |
|-----|--------|
| Ctrl+O | Open Config |
| Ctrl+S | Save Config |
| Ctrl+Shift+S | Save Config As |
| Home | Fit All in View |
| F | Focus Selected |
| G | Toggle Grid |
| T | Toggle TF Axes |
| Delete | Remove Selected Display |
| Escape | Deselect |
| F11 | Toggle Fullscreen |
| Ctrl+Q | Quit |

## Dialogs ❌ 未実装

将来実装予定:

### Connection Dialog

```
┌─ Connect to Server ────────────────────────────┐
│                                                │
│  Server URL:                                   │
│  [ws://localhost:9090                     ]    │
│                                                │
│  ☐ Remember this connection                    │
│                                                │
│  Recent:                                       │
│  • ws://localhost:9090                         │
│  • ws://robot.local:9090                       │
│                                                │
│                    [Cancel]  [Connect]         │
└────────────────────────────────────────────────┘
```

### Preferences Dialog

```
┌─ Preferences ──────────────────────────────────┐
│                                                │
│  ┌─────────┐                                   │
│  │ General │  Theme: [Dark ▼]                  │
│  │ Render  │                                   │
│  │ Network │  ☑ Restore layout on startup      │
│  │         │  ☑ Auto-connect to last server    │
│  └─────────┘                                   │
│                                                │
│              [Reset to Defaults]               │
│                                                │
│                    [Cancel]  [Apply]  [OK]     │
└────────────────────────────────────────────────┘
```

**現在の接続設定**: アプリケーション起動時に `ws://localhost:9090/ws` に自動接続

## Responsive Behavior ❌ 未実装

将来実装予定:

### Minimum Window Size
- Width: 800px
- Height: 600px

### Panel Collapse Behavior
| Window Width | Behavior |
|--------------|----------|
| < 1200px | 下部パネルを折りたたみ |
| < 1000px | 左右パネルをタブ化 |
| < 800px | パネルをオーバーレイモードに |

### Web-specific
- タッチ操作: 1本指ドラッグ=回転、2本指=パン、ピンチ=ズーム
- モバイル: 縦画面時は3Dビューフルスクリーン + ハンバーガーメニュー

## State Persistence ❌ 未実装

将来実装予定:

### Auto-saved (localStorage / file)
- ウィンドウサイズ・位置
- パネルレイアウト
- 最後の接続先
- テーマ設定

### Config File (.toml)
- Display設定
- 表示中のトピック
- カメラ位置
- Fixed frame

## Loading States

### Initial Connection
```
┌─────────────────────────────────────┐
│                                     │
│         ◐  Connecting...            │
│                                     │
│    ws://localhost:9090              │
│                                     │
│         [Cancel]                    │
└─────────────────────────────────────┘
```

### Data Loading
- 点群: プログレッシブ表示（受信した分から描画）
- TF: スピナー表示後にツリー構築

## Error Handling

### Connection Error
```
┌─ Connection Failed ────────────────────────────┐
│                                                │
│  ⚠ Could not connect to server                 │
│                                                │
│  ws://localhost:9090                           │
│  Error: Connection refused                     │
│                                                │
│                    [Retry]  [Close]            │
└────────────────────────────────────────────────┘
```

### Topic Error
- 赤いバッジでDisplayパネルに表示
- ツールチップでエラー詳細

## Accessibility

- キーボードナビゲーション対応
- フォーカス可視化
- 適切なARIAラベル (Web)
- 最小コントラスト比 4.5:1

## Implementation Status

### 実装済み ✅
- Topics Panel (フィルタ、選択、追加機能)
- Displays Panel (表示設定、ON/OFF、削除)
- TF Tree Panel (階層表示、Fixed Frame設定)
- Performance Panel (FPS、メモリ、帯域幅)
- 3D Viewport (カメラ制御、座標軸表示)
- Dock型レイアウト (egui_dock)
- WebSocket接続管理
- メニューバー (File/Edit/View/Displays/Help)
- ツールバー (接続、Frame選択、Grid/TF表示)
- テーマ切り替え (Dark/Light)
- Aboutダイアログ
- パネル表示/非表示 (Viewメニュー)
- レイアウトリセット
- キーボードショートカット (G/T/1-4/Escape/Home/F11/Ctrl+Q)
- ショートカットダイアログ

### 未実装 ❌
- 設定永続化
- レスポンシブ対応

## Keyboard Shortcuts ✅ 実装済み

| Key | Action |
|-----|--------|
| Ctrl+Q | Quit |
| Escape | Close dialogs / Deselect |
| F11 | Toggle Fullscreen |
| G | Toggle Grid |
| T | Toggle TF Axes |
| Home | Fit All in View |
| 1 | Toggle Topics Panel |
| 2 | Toggle Displays Panel |
| 3 | Toggle TF Tree Panel |
| 4 | Toggle Performance Panel |

## Implementation Notes

### egui Integration
```rust
// Dockingはegui_dockを使用
use egui_dock::{DockArea, DockState, Style, TabViewer};

// パネル列挙型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PanelKind {
    Viewport,
    Topics,
    Displays,
    TfTree,
    Performance,
}

// TabViewer実装でパネルを描画
impl TabViewer for WizTabViewer {
    type Tab = PanelKind;

    fn ui(&mut self, ui: &mut egui::Ui, tab: &mut Self::Tab) {
        match tab {
            PanelKind::Viewport => viewport_panel(ui, &mut self.viewport_state),
            PanelKind::Topics => topics_panel(ui, &mut self.app_state),
            PanelKind::Displays => displays_panel(ui, &mut self.app_state),
            PanelKind::TfTree => tf_tree_panel(ui, &mut self.app_state),
            PanelKind::Performance => performance_panel(ui, &mut self.perf_state),
        }
    }

    fn title(&mut self, tab: &mut Self::Tab) -> egui::WidgetText {
        match tab {
            PanelKind::Viewport => "Viewport".into(),
            PanelKind::Topics => "Topics".into(),
            PanelKind::Displays => "Displays".into(),
            PanelKind::TfTree => "TF Tree".into(),
            PanelKind::Performance => "Performance".into(),
        }
    }
}
```

### App State
```rust
pub struct AppState {
    pub topics: Vec<TopicInfo>,
    pub displays: Vec<Display>,
    pub filter: String,
    pub actions: Vec<AppAction>,
}

pub enum AppAction {
    Subscribe { topic: String, msg_type: String },
    Unsubscribe { id: String },
    RefreshTopics,
}
```

### Display Configuration
```rust
pub struct Display {
    pub id: String,
    pub topic: String,
    pub display_type: DisplayType,
    pub visible: bool,
    pub expanded: bool,
    pub settings: DisplaySettings,
}

pub struct DisplaySettings {
    pub point_size: f32,      // default: 2.0
    pub alpha: f32,           // default: 1.0
    pub color: [f32; 4],      // RGBA
}
```

### Performance Metrics
```rust
pub struct PerformanceState {
    pub fps_history: VecDeque<f32>,      // 120 samples
    pub frame_time_history: VecDeque<f32>, // 60 samples
    pub point_count: u64,
    pub triangle_count: u64,
    pub latency_ms: f32,
    pub bandwidth_mbps: f32,
    pub messages_per_sec: u32,
    pub gpu_memory_mb: f32,
    pub cpu_memory_mb: f32,
}
```
