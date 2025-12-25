use egui_dock::{DockArea, DockState, NodeIndex, Style};
use parking_lot::Mutex;
use std::sync::Arc;
use wiz_core::TopicInfo;

use crate::panels::{DisplaysPanel, Panel, PerformancePanel, TfTreePanel, TopicsPanel, ViewportPanel};
use crate::ws_client::WsClient;

pub struct WizApp {
    dock_state: DockState<Box<dyn Panel>>,
    ws_client: Arc<Mutex<Option<WsClient>>>,
    connection_url: String,
    connected: bool,
    topics: Vec<TopicInfo>,
}

impl WizApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut dock_state = DockState::new(vec![Box::new(ViewportPanel::new()) as Box<dyn Panel>]);

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
            ws_client: Arc::new(Mutex::new(None)),
            connection_url: "ws://localhost:9090".to_string(),
            connected: false,
            topics: Vec::new(),
        }
    }

    fn render_toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            // Connection section
            ui.label("Server:");
            ui.add(egui::TextEdit::singleline(&mut self.connection_url).desired_width(200.0));

            if self.connected {
                if ui.button("Disconnect").clicked() {
                    self.disconnect();
                }
                ui.colored_label(egui::Color32::GREEN, "\u{25CF}");
            } else {
                if ui.button("Connect").clicked() {
                    self.connect();
                }
                ui.colored_label(egui::Color32::RED, "\u{25CF}");
            }

            ui.separator();

            // Frame selection
            ui.label("Frame:");
            egui::ComboBox::from_id_salt("fixed_frame")
                .selected_text("map")
                .show_ui(ui, |ui| {
                    ui.selectable_label(true, "map");
                    ui.selectable_label(false, "odom");
                    ui.selectable_label(false, "base_link");
                });

            ui.separator();

            // View options
            ui.checkbox(&mut true, "Grid");
            ui.checkbox(&mut true, "TF Axes");
        });
    }

    fn connect(&mut self) {
        // TODO: Implement WebSocket connection
        self.connected = true;
    }

    fn disconnect(&mut self) {
        // TODO: Implement WebSocket disconnection
        self.connected = false;
    }
}

impl eframe::App for WizApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
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
            DockArea::new(&mut self.dock_state)
                .style(Style::from_egui(ui.style().as_ref()))
                .show_inside(ui, &mut TabViewer);
        });
    }
}

struct TabViewer;

impl egui_dock::TabViewer for TabViewer {
    type Tab = Box<dyn Panel>;

    fn title(&mut self, tab: &mut Self::Tab) -> egui::WidgetText {
        tab.name().into()
    }

    fn ui(&mut self, ui: &mut egui::Ui, tab: &mut Self::Tab) {
        tab.ui(ui);
    }
}
