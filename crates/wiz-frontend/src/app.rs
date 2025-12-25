use egui_dock::{DockArea, DockState, NodeIndex, Style};
use parking_lot::Mutex;
use std::sync::Arc;
use tracing::info;
use wiz_core::TopicInfo;

use crate::panels::{DisplaysPanel, Panel, PerformancePanel, TfTreePanel, TopicsPanel, ViewportPanel};
use crate::viewport_state::{SharedViewportState, ViewportState};
use crate::ws_client::{ConnectionState, WsClientHandle, WsEvent};

pub struct WizApp {
    dock_state: DockState<Box<dyn Panel>>,
    ws_client: WsClientHandle,
    connection_url: String,
    topics: Vec<TopicInfo>,
    viewport_state: Option<SharedViewportState>,
    show_grid: bool,
    show_tf_axes: bool,
    fixed_frame: String,
    status_message: Option<String>,
}

impl WizApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Initialize viewport state if wgpu is available
        let viewport_state = cc.wgpu_render_state.as_ref().map(|render_state| {
            let device = render_state.device.clone();
            let queue = render_state.queue.clone();
            let format = render_state.target_format;

            let mut state = ViewportState::new(device, queue, format);
            // Add sample data for testing
            state.add_sample_point_cloud();
            state.add_sample_laser_scan();

            Arc::new(Mutex::new(state))
        });

        // Create viewport panel with shared state
        let viewport_panel = ViewportPanel::new(viewport_state.clone());

        let mut dock_state = DockState::new(vec![Box::new(viewport_panel) as Box<dyn Panel>]);

        let [main, _right] = dock_state.main_surface_mut().split_right(
            NodeIndex::root(),
            0.75,
            vec![Box::new(DisplaysPanel::new()) as Box<dyn Panel>],
        );

        let [_left, _main] = dock_state.main_surface_mut().split_left(
            main,
            0.2,
            vec![Box::new(TopicsPanel::new()) as Box<dyn Panel>],
        );

        let [_main, _bottom] = dock_state.main_surface_mut().split_below(
            main,
            0.7,
            vec![
                Box::new(TfTreePanel::new()) as Box<dyn Panel>,
                Box::new(PerformancePanel::new()) as Box<dyn Panel>,
            ],
        );

        Self {
            dock_state,
            ws_client: WsClientHandle::new(),
            connection_url: "ws://localhost:9090/ws".to_string(),
            topics: Vec::new(),
            viewport_state,
            show_grid: true,
            show_tf_axes: true,
            fixed_frame: "map".to_string(),
            status_message: None,
        }
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
                    ui.colored_label(egui::Color32::RED, format!("\u{25CF} {}", msg));
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
                        if ui.selectable_label(self.fixed_frame == frame, frame).clicked() {
                            self.fixed_frame = frame.to_string();
                        }
                    }
                });

            ui.separator();

            // View options
            ui.checkbox(&mut self.show_grid, "Grid");
            ui.checkbox(&mut self.show_tf_axes, "TF Axes");

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
                    self.topics.clear();
                }
                WsEvent::Message(msg) => {
                    self.handle_server_message(msg);
                }
                WsEvent::Error(e) => {
                    self.status_message = Some(format!("Error: {}", e));
                }
            }
        }
    }

    fn handle_server_message(&mut self, msg: wiz_protocol::ServerMessage) {
        use wiz_protocol::ServerMessage;
        match msg {
            ServerMessage::Topics { topics } => {
                info!("Received {} topics", topics.len());
                self.topics = topics;
            }
            ServerMessage::Subscribed { id, topic } => {
                info!("Subscribed to {} (id: {})", topic, id);
            }
            ServerMessage::Unsubscribed { id } => {
                info!("Unsubscribed (id: {})", id);
            }
            ServerMessage::Data { topic, msg_type, timestamp, payload } => {
                // TODO: Handle incoming data (point clouds, laser scans, etc.)
                info!("Received data from {} ({}) at {}: {} bytes", topic, msg_type, timestamp, payload.len());
            }
            ServerMessage::Transform { target_frame, source_frame, transform } => {
                info!("TF: {} -> {} = {:?}", source_frame, target_frame, transform);
            }
            ServerMessage::Error { message } => {
                self.status_message = Some(format!("Server error: {}", message));
            }
        }
    }
}

impl eframe::App for WizApp {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        // Process WebSocket events
        self.process_ws_events();

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
                    if ui.button("Exit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });

                ui.menu_button("Edit", |ui| {
                    if ui.button("Preferences...").clicked() {
                        ui.close_menu();
                    }
                });

                ui.menu_button("View", |ui| {
                    if ui.button("Topics Panel").clicked() {
                        ui.close_menu();
                    }
                    if ui.button("Displays Panel").clicked() {
                        ui.close_menu();
                    }
                    if ui.button("TF Tree Panel").clicked() {
                        ui.close_menu();
                    }
                    if ui.button("Performance Panel").clicked() {
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Reset Layout").clicked() {
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
                        ui.close_menu();
                    }
                    if ui.button("About wiz").clicked() {
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
        if tab.name() == "3D Viewport" {
            if let (Some(render_state), Some(viewport_state)) =
                (self.render_state, &self.viewport_state)
            {
                tab.ui_with_render_context(ui, render_state, viewport_state.clone());
                return;
            }
        }
        tab.ui(ui);
    }
}
