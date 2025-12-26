use egui_dock::{DockArea, DockState, NodeIndex, Style};
use parking_lot::Mutex;
use std::sync::Arc;
use tracing::info;

use crate::app_state::{AppAction, AppState, SharedAppState};
use crate::panels::{Panel, PerformancePanel, PropertyPanel, TopicsPanel, ViewportPanel};
use crate::viewport_state::{SharedViewportState, ViewportState};
use crate::ws_client::{ConnectionState, WsClientHandle, WsEvent};

/// Tracks which panels are currently visible/open
struct PanelVisibility {
    topics: bool,
    property: bool,
    performance: bool,
}

impl Default for PanelVisibility {
    fn default() -> Self {
        Self {
            topics: true,
            property: true,
            performance: true,
        }
    }
}

pub struct WizApp {
    dock_state: DockState<Box<dyn Panel>>,
    ws_client: WsClientHandle,
    connection_url: String,
    app_state: SharedAppState,
    viewport_state: Option<SharedViewportState>,
    show_grid: bool,
    show_tf_axes: bool,
    fixed_frame: String,
    status_message: Option<String>,
    /// Panel visibility state
    panel_visibility: PanelVisibility,
    /// Show about dialog
    show_about_dialog: bool,
    /// Show keyboard shortcuts dialog
    show_shortcuts_dialog: bool,
}

impl WizApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Create shared app state
        let app_state = Arc::new(Mutex::new(AppState::new()));

        // Initialize viewport state if wgpu is available
        let viewport_state = cc.wgpu_render_state.as_ref().map(|render_state| {
            let device = render_state.device.clone();
            let queue = render_state.queue.clone();
            let format = render_state.target_format;

            let state = ViewportState::new(device, queue, format);
            Arc::new(Mutex::new(state))
        });

        // Create viewport panel with shared state
        let viewport_panel = ViewportPanel::new(viewport_state.clone());

        let mut dock_state = DockState::new(vec![Box::new(viewport_panel) as Box<dyn Panel>]);

        let [main, _right] = dock_state.main_surface_mut().split_right(
            NodeIndex::root(),
            0.75,
            vec![Box::new(PropertyPanel::new(app_state.clone())) as Box<dyn Panel>],
        );

        let [_left, _main] = dock_state.main_surface_mut().split_left(
            main,
            0.2,
            vec![Box::new(TopicsPanel::new(app_state.clone())) as Box<dyn Panel>],
        );

        let [_main, _bottom] = dock_state.main_surface_mut().split_below(
            main,
            0.7,
            vec![Box::new(PerformancePanel::new(app_state.clone())) as Box<dyn Panel>],
        );

        Self {
            dock_state,
            ws_client: WsClientHandle::new(),
            connection_url: "ws://localhost:9090/ws".to_string(),
            app_state,
            viewport_state,
            show_grid: true,
            show_tf_axes: true,
            fixed_frame: "map".to_string(),
            status_message: None,
            panel_visibility: PanelVisibility::default(),
            show_about_dialog: false,
            show_shortcuts_dialog: false,
        }
    }

    /// Check if a panel with the given name exists in the dock
    fn has_panel(&self, name: &str) -> bool {
        for ((_surface_idx, _node_idx), tab) in self.dock_state.iter_all_tabs() {
            if tab.name() == name {
                return true;
            }
        }
        false
    }

    /// Add a panel to the dock
    fn add_panel(&mut self, panel: Box<dyn Panel>) {
        self.dock_state.push_to_first_leaf(panel);
    }

    /// Remove a panel from the dock by name
    fn remove_panel(&mut self, name: &str) {
        // Use retain_tabs to remove the panel by name
        self.dock_state.retain_tabs(|tab| tab.name() != name);
    }

    /// Sync panel visibility state with actual dock state
    fn sync_panel_visibility(&mut self) {
        self.panel_visibility.topics = self.has_panel("Topics");
        self.panel_visibility.property = self.has_panel("Property");
        self.panel_visibility.performance = self.has_panel("Performance");
    }

    /// Toggle a panel's visibility
    fn toggle_panel(&mut self, name: &str, should_be_visible: bool) {
        let is_visible = self.has_panel(name);

        if should_be_visible && !is_visible {
            // Add the panel
            let panel: Box<dyn Panel> = match name {
                "Topics" => Box::new(TopicsPanel::new(self.app_state.clone())),
                "Property" => Box::new(PropertyPanel::new(self.app_state.clone())),
                "Performance" => Box::new(PerformancePanel::new(self.app_state.clone())),
                _ => return,
            };
            self.add_panel(panel);
        } else if !should_be_visible && is_visible {
            self.remove_panel(name);
        }

        // Update visibility state
        match name {
            "Topics" => self.panel_visibility.topics = should_be_visible,
            "Property" => self.panel_visibility.property = should_be_visible,
            "Performance" => self.panel_visibility.performance = should_be_visible,
            _ => {}
        }
    }

    /// Show About dialog
    fn show_about_dialog(ctx: &egui::Context, open: &mut bool) {
        egui::Window::new("About wiz")
            .open(open)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.vertical_centered(|ui| {
                    ui.heading("wiz");
                    ui.add_space(8.0);
                    ui.label("ROS2 Visualization Tool");
                    ui.add_space(4.0);
                    ui.label(format!("Version: {}", env!("CARGO_PKG_VERSION")));
                    ui.add_space(8.0);
                    ui.separator();
                    ui.add_space(8.0);
                    ui.label("A next-generation visualization tool for ROS2");
                    ui.label("Built with Rust, WebGPU, and egui");
                    ui.add_space(8.0);
                    ui.hyperlink_to("GitHub Repository", "https://github.com/NOPLAB/wiz");
                    ui.add_space(8.0);
                    ui.label("MIT License");
                });
            });
    }

    /// Show Keyboard Shortcuts dialog
    fn show_shortcuts_dialog(ctx: &egui::Context, open: &mut bool) {
        egui::Window::new("Keyboard Shortcuts")
            .open(open)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                egui::Grid::new("shortcuts_grid")
                    .num_columns(2)
                    .spacing([40.0, 6.0])
                    .show(ui, |ui| {
                        ui.label(egui::RichText::new("General").strong());
                        ui.end_row();

                        ui.label("Ctrl+Q");
                        ui.label("Quit");
                        ui.end_row();

                        ui.label("Escape");
                        ui.label("Close dialogs / Deselect");
                        ui.end_row();

                        ui.label("F11");
                        ui.label("Toggle Fullscreen");
                        ui.end_row();

                        ui.label("");
                        ui.end_row();

                        ui.label(egui::RichText::new("View").strong());
                        ui.end_row();

                        ui.label("G");
                        ui.label("Toggle Grid");
                        ui.end_row();

                        ui.label("T");
                        ui.label("Toggle TF Axes");
                        ui.end_row();

                        ui.label("Home");
                        ui.label("Fit All in View");
                        ui.end_row();

                        ui.label("");
                        ui.end_row();

                        ui.label(egui::RichText::new("Panels").strong());
                        ui.end_row();

                        ui.label("1");
                        ui.label("Toggle Topics Panel");
                        ui.end_row();

                        ui.label("2");
                        ui.label("Toggle Property Panel");
                        ui.end_row();

                        ui.label("3");
                        ui.label("Toggle Performance Panel");
                        ui.end_row();
                    });
            });
    }

    /// Reset to default layout
    fn reset_layout(&mut self) {
        // Create viewport panel with shared state
        let viewport_panel = ViewportPanel::new(self.viewport_state.clone());

        let mut dock_state = DockState::new(vec![Box::new(viewport_panel) as Box<dyn Panel>]);

        let [main, _right] = dock_state.main_surface_mut().split_right(
            NodeIndex::root(),
            0.75,
            vec![Box::new(PropertyPanel::new(self.app_state.clone())) as Box<dyn Panel>],
        );

        let [_left, _main] = dock_state.main_surface_mut().split_left(
            main,
            0.2,
            vec![Box::new(TopicsPanel::new(self.app_state.clone())) as Box<dyn Panel>],
        );

        let [_main, _bottom] = dock_state.main_surface_mut().split_below(
            main,
            0.7,
            vec![Box::new(PerformancePanel::new(self.app_state.clone())) as Box<dyn Panel>],
        );

        self.dock_state = dock_state;
        self.panel_visibility = PanelVisibility::default();
    }

    /// Handle keyboard shortcuts
    fn handle_keyboard_shortcuts(&mut self, ctx: &egui::Context) {
        ctx.input_mut(|input| {
            // Ctrl+Q: Quit
            if input.consume_key(egui::Modifiers::COMMAND, egui::Key::Q) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }

            // G: Toggle Grid
            if input.consume_key(egui::Modifiers::NONE, egui::Key::G) {
                self.show_grid = !self.show_grid;
            }

            // T: Toggle TF Axes
            if input.consume_key(egui::Modifiers::NONE, egui::Key::T) {
                self.show_tf_axes = !self.show_tf_axes;
                if let Some(ref state) = self.viewport_state {
                    state.lock().set_show_tf_frames(self.show_tf_axes);
                }
            }

            // Home: Fit All in View
            if input.consume_key(egui::Modifiers::NONE, egui::Key::Home) {
                if let Some(ref state) = self.viewport_state {
                    state.lock().renderer.camera.fit_all(glam::Vec3::ZERO, 5.0);
                }
            }

            // Escape: Deselect / Close dialogs
            if input.consume_key(egui::Modifiers::NONE, egui::Key::Escape) {
                self.show_about_dialog = false;
                self.show_shortcuts_dialog = false;
                self.app_state.lock().selected_topic = None;
            }

            // F11: Toggle Fullscreen
            if input.consume_key(egui::Modifiers::NONE, egui::Key::F11) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(true));
            }

            // 1: Topics Panel toggle
            if input.consume_key(egui::Modifiers::NONE, egui::Key::Num1) {
                let visible = !self.has_panel("Topics");
                self.toggle_panel("Topics", visible);
            }

            // 2: Property Panel toggle
            if input.consume_key(egui::Modifiers::NONE, egui::Key::Num2) {
                let visible = !self.has_panel("Property");
                self.toggle_panel("Property", visible);
            }

            // 3: Performance Panel toggle
            if input.consume_key(egui::Modifiers::NONE, egui::Key::Num3) {
                let visible = !self.has_panel("Performance");
                self.toggle_panel("Performance", visible);
            }

            // R: Reset Layout
            if input.consume_key(egui::Modifiers::NONE, egui::Key::R) {
                // Only reset if not typing in a text field
                // This is handled by input_mut consuming the key
            }
        });
    }

    fn render_toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            // Connection section
            ui.label("Server:");
            ui.add(egui::TextEdit::singleline(&mut self.connection_url).desired_width(200.0));

            let state = self.ws_client.state();
            match state {
                ConnectionState::Connected => {
                    if ui.button("Disconnect").clicked() {
                        self.disconnect();
                    }
                    ui.colored_label(egui::Color32::GREEN, "\u{25CF} Connected");
                }
                ConnectionState::Connecting => {
                    ui.add_enabled(false, egui::Button::new("Connecting..."));
                    ui.colored_label(egui::Color32::YELLOW, "\u{25CF}");
                }
                ConnectionState::Disconnected => {
                    if ui.button("Connect").clicked() {
                        self.connect();
                    }
                    ui.colored_label(egui::Color32::RED, "\u{25CF}");
                }
                ConnectionState::Error(ref msg) => {
                    if ui.button("Retry").clicked() {
                        self.connect();
                    }
                    ui.colored_label(egui::Color32::RED, format!("\u{25CF} {msg}"));
                }
            }

            ui.separator();

            // Frame selection
            ui.label("Frame:");
            egui::ComboBox::from_id_salt("fixed_frame")
                .selected_text(&self.fixed_frame)
                .show_ui(ui, |ui| {
                    let frames = ["map", "odom", "base_link"];
                    for frame in frames {
                        if ui
                            .selectable_label(self.fixed_frame == frame, frame)
                            .clicked()
                        {
                            self.fixed_frame = frame.to_string();
                        }
                    }
                });

            ui.separator();

            // View options
            ui.checkbox(&mut self.show_grid, "Grid");
            if ui.checkbox(&mut self.show_tf_axes, "TF Axes").changed() {
                if let Some(ref state) = self.viewport_state {
                    state.lock().set_show_tf_frames(self.show_tf_axes);
                }
            }

            // Topic count
            let topic_count = self.app_state.lock().topics.len();
            if topic_count > 0 {
                ui.separator();
                ui.label(format!("{topic_count} topics"));
            }

            // Status message
            if let Some(ref msg) = self.status_message {
                ui.separator();
                ui.label(msg);
            }
        });
    }

    fn connect(&mut self) {
        info!("Connecting to {}", self.connection_url);
        self.ws_client.connect(&self.connection_url);
    }

    fn disconnect(&mut self) {
        info!("Disconnecting");
        self.ws_client.disconnect();
        // Clear topics on disconnect
        self.app_state.lock().set_topics(Vec::new());
    }

    fn process_ws_events(&mut self) {
        for event in self.ws_client.poll_events() {
            match event {
                WsEvent::Connected => {
                    self.status_message = Some("Connected".to_string());
                    // Request topic list after connection
                    self.ws_client.list_topics();
                }
                WsEvent::Disconnected => {
                    self.status_message = Some("Disconnected".to_string());
                    self.app_state.lock().set_topics(Vec::new());
                }
                WsEvent::Message(msg) => {
                    self.handle_server_message(msg);
                }
                WsEvent::Error(e) => {
                    self.status_message = Some(format!("Error: {e}"));
                }
            }
        }
    }

    fn handle_server_message(&mut self, msg: wiz_protocol::ServerMessage) {
        use wiz_protocol::ServerMessage;
        match msg {
            ServerMessage::Topics { topics } => {
                info!("Received {} topics", topics.len());
                self.app_state.lock().set_topics(topics);
            }
            ServerMessage::Subscribed { id, topic } => {
                info!("Subscribed to {} (id: {})", topic, id);
            }
            ServerMessage::Unsubscribed { id } => {
                info!("Unsubscribed (id: {})", id);
            }
            ServerMessage::Data {
                topic,
                msg_type,
                timestamp,
                payload,
            } => {
                info!(
                    "Received data from {} ({}) at {}: {} bytes",
                    topic,
                    msg_type,
                    timestamp,
                    payload.len()
                );
                self.handle_data_message(&topic, &msg_type, timestamp, &payload);
            }
            ServerMessage::Transform {
                target_frame,
                source_frame,
                transform,
            } => {
                info!("TF: {} -> {} = {:?}", source_frame, target_frame, transform);
            }
            ServerMessage::Error { message } => {
                self.status_message = Some(format!("Server error: {message}"));
            }
        }
    }

    fn process_pending_actions(&mut self) {
        let actions = self.app_state.lock().take_pending_actions();
        for action in actions {
            match action {
                AppAction::Subscribe { topic, msg_type } => {
                    info!("Subscribing to {} ({})", topic, msg_type);
                    self.ws_client.subscribe(&topic, &msg_type, None);
                }
                AppAction::Unsubscribe { subscription_id } => {
                    info!("Unsubscribing from subscription {}", subscription_id);
                    self.ws_client.unsubscribe(subscription_id);
                }
                AppAction::RefreshTopics => {
                    info!("Refreshing topic list");
                    self.ws_client.list_topics();
                }
            }
        }
    }

    /// Apply display settings from AppState to ViewportState
    fn apply_display_settings(&mut self) {
        let Some(ref viewport_state) = self.viewport_state else {
            return;
        };

        let state = self.app_state.lock();
        let mut viewport = viewport_state.lock();

        // Apply visibility and settings for each display
        for display in &state.displays {
            // Sync visibility to ViewportState
            viewport.set_topic_visibility(&display.topic, display.visible);

            // Apply other settings only for visible displays
            if !display.visible {
                continue;
            }
            match display.display_type {
                crate::panels::property::DisplayType::PointCloud2 => {
                    viewport.set_point_size(display.point_size);
                    viewport.set_point_alpha(display.alpha);
                }
                crate::panels::property::DisplayType::LaserScan => {
                    viewport.set_laser_scan_color(display.color);
                }
                crate::panels::property::DisplayType::Pose => {
                    viewport.set_pose_color(display.color);
                    viewport.set_pose_arrow_length(display.arrow_length);
                    viewport.set_pose_arrow_width(display.arrow_width);
                }
                _ => {}
            }
        }
    }

    fn handle_data_message(&mut self, topic: &str, msg_type: &str, timestamp: f64, payload: &[u8]) {
        // Record network stats and latency
        {
            let mut state = self.app_state.lock();
            state.performance_stats.record_data(payload.len() as u64, 1);
            state.performance_stats.record_latency(timestamp);
        }

        let Some(ref viewport_state) = self.viewport_state else {
            return;
        };

        // Decode payload based on message type
        if msg_type.contains("PointCloud2") {
            match rmp_serde::from_slice::<wiz_core::PointCloud2>(payload) {
                Ok(cloud) => {
                    let point_count = cloud.point_count();
                    tracing::debug!("Updating point cloud {} with {} points", topic, point_count);
                    viewport_state.lock().update_point_cloud(topic, &cloud);

                    // Update GPU memory estimate (position: 12 bytes + color: 4 bytes = 16 bytes per point)
                    self.app_state
                        .lock()
                        .performance_stats
                        .update_gpu_memory(point_count, 16);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode PointCloud2: {}", e);
                }
            }
        } else if msg_type.contains("LaserScan") {
            match rmp_serde::from_slice::<wiz_core::LaserScan>(payload) {
                Ok(scan) => {
                    tracing::debug!(
                        "Updating laser scan {} with {} ranges",
                        topic,
                        scan.ranges.len()
                    );
                    viewport_state.lock().update_laser_scan(topic, &scan);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode LaserScan: {}", e);
                }
            }
        } else if msg_type.contains("PoseStamped") {
            match rmp_serde::from_slice::<wiz_core::PoseStamped>(payload) {
                Ok(pose) => {
                    tracing::debug!("Updating pose {} at {:?}", topic, pose.pose.position);
                    viewport_state.lock().update_pose(topic, &pose);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode PoseStamped: {}", e);
                }
            }
        } else if msg_type.contains("MarkerArray") {
            match rmp_serde::from_slice::<wiz_core::MarkerArray>(payload) {
                Ok(marker_array) => {
                    tracing::debug!(
                        "Updating markers {} with {} markers",
                        topic,
                        marker_array.markers.len()
                    );
                    viewport_state.lock().update_markers(topic, &marker_array);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode MarkerArray: {}", e);
                }
            }
        } else if msg_type.contains("TFMessage") {
            match rmp_serde::from_slice::<wiz_core::TFMessage>(payload) {
                Ok(tf_message) => {
                    tracing::debug!(
                        "Updating TF {} with {} transforms",
                        topic,
                        tf_message.transforms.len()
                    );
                    viewport_state.lock().update_tf_message(topic, &tf_message);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode TFMessage: {}", e);
                }
            }
        } else if msg_type.contains("Odometry") {
            match rmp_serde::from_slice::<wiz_core::Odometry>(payload) {
                Ok(odom) => {
                    tracing::debug!(
                        "Updating odometry {} at {:?}",
                        topic,
                        odom.pose.pose.position
                    );
                    viewport_state.lock().update_odometry(topic, &odom);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode Odometry: {}", e);
                }
            }
        } else if msg_type.contains("Imu") {
            match rmp_serde::from_slice::<wiz_core::Imu>(payload) {
                Ok(imu) => {
                    tracing::debug!("Updating IMU {} orientation {:?}", topic, imu.orientation);
                    viewport_state.lock().update_imu(topic, &imu);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode Imu: {}", e);
                }
            }
        } else if msg_type.contains("Twist") {
            match rmp_serde::from_slice::<wiz_core::TwistStamped>(payload) {
                Ok(twist) => {
                    tracing::debug!("Updating twist {} linear {:?}", topic, twist.twist.linear);
                    viewport_state.lock().update_twist(topic, &twist);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode TwistStamped: {}", e);
                }
            }
        } else if msg_type.contains("JointState") {
            match rmp_serde::from_slice::<wiz_core::JointState>(payload) {
                Ok(joint_state) => {
                    tracing::debug!(
                        "Updating joint state {} with {} joints",
                        topic,
                        joint_state.name.len()
                    );
                    viewport_state
                        .lock()
                        .update_joint_state(topic, &joint_state);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode JointState: {}", e);
                }
            }
        } else if msg_type.contains("CameraInfo") {
            match rmp_serde::from_slice::<wiz_core::CameraInfo>(payload) {
                Ok(camera_info) => {
                    tracing::debug!(
                        "Updating camera info {} ({}x{})",
                        topic,
                        camera_info.width,
                        camera_info.height
                    );
                    viewport_state
                        .lock()
                        .update_camera_info(topic, &camera_info);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode CameraInfo: {}", e);
                }
            }
        } else if msg_type.contains("Clock") {
            match rmp_serde::from_slice::<wiz_core::Clock>(payload) {
                Ok(clock) => {
                    let text = format!("{:.3} sec", clock.clock);
                    self.app_state.lock().update_display_text(topic, text, None);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode Clock: {}", e);
                }
            }
        } else if msg_type.contains("String") {
            match rmp_serde::from_slice::<wiz_core::StringMsg>(payload) {
                Ok(string_msg) => {
                    self.app_state
                        .lock()
                        .update_display_text(topic, string_msg.data, None);
                }
                Err(e) => {
                    tracing::warn!("Failed to decode String: {}", e);
                }
            }
        } else if msg_type.contains("Log") {
            match rmp_serde::from_slice::<wiz_core::Log>(payload) {
                Ok(log) => {
                    let level_str = match log.level {
                        wiz_core::LogLevel::Debug => "DEBUG",
                        wiz_core::LogLevel::Info => "INFO",
                        wiz_core::LogLevel::Warn => "WARN",
                        wiz_core::LogLevel::Error => "ERROR",
                        wiz_core::LogLevel::Fatal => "FATAL",
                    };
                    let text = format!("[{}] {}", log.name, log.msg);
                    self.app_state.lock().update_display_text(
                        topic,
                        text,
                        Some(level_str.to_string()),
                    );
                }
                Err(e) => {
                    tracing::warn!("Failed to decode Log: {}", e);
                }
            }
        }
    }
}

impl eframe::App for WizApp {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        // Process WebSocket events
        self.process_ws_events();

        // Process pending actions from UI
        self.process_pending_actions();

        // Apply display settings to renderer
        self.apply_display_settings();

        // Update performance stats
        self.app_state.lock().performance_stats.update();

        // Handle keyboard shortcuts
        self.handle_keyboard_shortcuts(ctx);

        // Menu bar
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("New Layout").clicked() {
                        ui.close_menu();
                    }
                    if ui.button("Open Config...").clicked() {
                        ui.close_menu();
                    }
                    if ui.button("Save Config").clicked() {
                        ui.close_menu();
                    }
                    ui.separator();
                    ui.horizontal(|ui| {
                        if ui.button("Exit").clicked() {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(egui::RichText::new("Ctrl+Q").weak());
                        });
                    });
                });

                ui.menu_button("Edit", |ui| {
                    if ui.button("Preferences...").clicked() {
                        ui.close_menu();
                    }
                });

                ui.menu_button("View", |ui| {
                    // Sync visibility state with actual dock state
                    self.sync_panel_visibility();

                    let mut topics_visible = self.panel_visibility.topics;
                    let mut property_visible = self.panel_visibility.property;
                    let mut performance_visible = self.panel_visibility.performance;

                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut topics_visible, "Topics Panel").clicked() {
                            self.toggle_panel("Topics", topics_visible);
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(egui::RichText::new("1").weak());
                        });
                    });
                    ui.horizontal(|ui| {
                        if ui
                            .checkbox(&mut property_visible, "Property Panel")
                            .clicked()
                        {
                            self.toggle_panel("Property", property_visible);
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(egui::RichText::new("2").weak());
                        });
                    });
                    ui.horizontal(|ui| {
                        if ui
                            .checkbox(&mut performance_visible, "Performance Panel")
                            .clicked()
                        {
                            self.toggle_panel("Performance", performance_visible);
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(egui::RichText::new("3").weak());
                        });
                    });
                    ui.separator();
                    if ui.button("Reset Layout").clicked() {
                        // Reset to default layout
                        self.reset_layout();
                        ui.close_menu();
                    }
                    ui.separator();
                    ui.menu_button("Theme", |ui| {
                        if ui.button("Dark").clicked() {
                            ctx.set_visuals(egui::Visuals::dark());
                            ui.close_menu();
                        }
                        if ui.button("Light").clicked() {
                            ctx.set_visuals(egui::Visuals::light());
                            ui.close_menu();
                        }
                    });
                });

                ui.menu_button("Displays", |ui| {
                    if ui.button("Add PointCloud2").clicked() {
                        ui.close_menu();
                    }
                    if ui.button("Add LaserScan").clicked() {
                        ui.close_menu();
                    }
                    if ui.button("Add TF").clicked() {
                        ui.close_menu();
                    }
                    if ui.button("Add Marker").clicked() {
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Remove All").clicked() {
                        ui.close_menu();
                    }
                });

                ui.menu_button("Help", |ui| {
                    if ui.button("Keyboard Shortcuts").clicked() {
                        self.show_shortcuts_dialog = true;
                        ui.close_menu();
                    }
                    if ui.button("About wiz").clicked() {
                        self.show_about_dialog = true;
                        ui.close_menu();
                    }
                });
            });
        });

        // Toolbar
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            self.render_toolbar(ui);
        });

        // Dock area
        egui::CentralPanel::default().show(ctx, |ui| {
            // Create TabViewer with render state reference
            let render_state = frame.wgpu_render_state();
            let mut tab_viewer = TabViewer {
                render_state,
                viewport_state: self.viewport_state.clone(),
            };

            DockArea::new(&mut self.dock_state)
                .style(Style::from_egui(ui.style().as_ref()))
                .show_inside(ui, &mut tab_viewer);
        });

        // Show dialogs if requested
        if self.show_about_dialog {
            Self::show_about_dialog(ctx, &mut self.show_about_dialog);
        }
        if self.show_shortcuts_dialog {
            Self::show_shortcuts_dialog(ctx, &mut self.show_shortcuts_dialog);
        }

        // Request continuous repaint for smooth rendering
        ctx.request_repaint();
    }
}

struct TabViewer<'a> {
    render_state: Option<&'a egui_wgpu::RenderState>,
    viewport_state: Option<SharedViewportState>,
}

impl egui_dock::TabViewer for TabViewer<'_> {
    type Tab = Box<dyn Panel>;

    fn title(&mut self, tab: &mut Self::Tab) -> egui::WidgetText {
        tab.name().into()
    }

    fn ui(&mut self, ui: &mut egui::Ui, tab: &mut Self::Tab) {
        // Pass render context to viewport panel if it needs it
        if tab.name() == "3D Viewport"
            && let (Some(render_state), Some(viewport_state)) =
                (self.render_state, &self.viewport_state)
        {
            tab.ui_with_render_context(ui, render_state, viewport_state.clone());
            return;
        }
        tab.ui(ui);
    }
}
