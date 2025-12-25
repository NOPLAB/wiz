use crate::app_state::SharedAppState;
use crate::panels::Panel;

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub enum DisplayType {
    PointCloud2,
    LaserScan,
    TF,
    Marker,
    Path,
    Pose,
}

#[derive(Debug, Clone)]
pub struct Display {
    pub display_type: DisplayType,
    pub topic: String,
    pub visible: bool,
    pub expanded: bool,
    pub point_size: f32,
    pub alpha: f32,
    pub color: [f32; 4],
    pub subscription_id: u32,
    /// Arrow length for Pose displays
    pub arrow_length: f32,
    /// Arrow width for Pose displays
    pub arrow_width: f32,
}

impl Display {
    pub fn new(display_type: DisplayType, topic: String, subscription_id: u32) -> Self {
        let (color, arrow_length, arrow_width) = match display_type {
            DisplayType::Pose => ([1.0, 0.0, 0.5, 1.0], 1.0, 0.1), // Magenta
            DisplayType::LaserScan => ([1.0, 0.3, 0.3, 1.0], 1.0, 0.1), // Red
            _ => ([1.0, 0.0, 0.0, 1.0], 1.0, 0.1),
        };
        Self {
            display_type,
            topic,
            visible: true,
            expanded: true,
            point_size: 2.0,
            alpha: 1.0,
            color,
            subscription_id,
            arrow_length,
            arrow_width,
        }
    }
}

pub struct DisplaysPanel {
    app_state: SharedAppState,
}

impl DisplaysPanel {
    pub fn new(app_state: SharedAppState) -> Self {
        Self { app_state }
    }
}

impl Panel for DisplaysPanel {
    fn name(&self) -> &str {
        "Displays"
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        let mut to_remove = None;

        egui::ScrollArea::vertical().show(ui, |ui| {
            let mut state = self.app_state.lock();

            if state.displays.is_empty() {
                ui.label("No topic displays added.");
                ui.separator();
                ui.label("To add a display:");
                ui.label("1. Connect to server");
                ui.label("2. Select a topic from Topics panel");
                ui.label("3. Double-click or use menu below");
                ui.separator();
                ui.label("Note: Grid and TF axes are");
                ui.label("controlled via toolbar settings.");
            }

            for (i, display) in state.displays.iter_mut().enumerate() {
                let type_name = match display.display_type {
                    DisplayType::PointCloud2 => "PointCloud2",
                    DisplayType::LaserScan => "LaserScan",
                    DisplayType::TF => "TF",
                    DisplayType::Marker => "Marker",
                    DisplayType::Path => "Path",
                    DisplayType::Pose => "Pose",
                };

                let header =
                    egui::CollapsingHeader::new(format!("{}: {}", type_name, display.topic))
                        .default_open(display.expanded)
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                ui.checkbox(&mut display.visible, "Visible");
                            });

                            ui.horizontal(|ui| {
                                ui.label("Topic:");
                                ui.label(&display.topic);
                            });

                            match display.display_type {
                                DisplayType::PointCloud2 => {
                                    ui.horizontal(|ui| {
                                        ui.label("Size:");
                                        ui.add(egui::Slider::new(
                                            &mut display.point_size,
                                            0.5..=10.0,
                                        ));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Alpha:");
                                        ui.add(egui::Slider::new(&mut display.alpha, 0.0..=1.0));
                                    });
                                }
                                DisplayType::LaserScan => {
                                    ui.horizontal(|ui| {
                                        ui.label("Color:");
                                        let mut color = egui::Color32::from_rgba_unmultiplied(
                                            (display.color[0] * 255.0) as u8,
                                            (display.color[1] * 255.0) as u8,
                                            (display.color[2] * 255.0) as u8,
                                            (display.color[3] * 255.0) as u8,
                                        );
                                        if egui::color_picker::color_edit_button_srgba(
                                            ui,
                                            &mut color,
                                            egui::color_picker::Alpha::Opaque,
                                        )
                                        .changed()
                                        {
                                            display.color = [
                                                color.r() as f32 / 255.0,
                                                color.g() as f32 / 255.0,
                                                color.b() as f32 / 255.0,
                                                color.a() as f32 / 255.0,
                                            ];
                                        }
                                    });
                                }
                                DisplayType::Pose => {
                                    ui.horizontal(|ui| {
                                        ui.label("Color:");
                                        let mut color = egui::Color32::from_rgba_unmultiplied(
                                            (display.color[0] * 255.0) as u8,
                                            (display.color[1] * 255.0) as u8,
                                            (display.color[2] * 255.0) as u8,
                                            (display.color[3] * 255.0) as u8,
                                        );
                                        if egui::color_picker::color_edit_button_srgba(
                                            ui,
                                            &mut color,
                                            egui::color_picker::Alpha::Opaque,
                                        )
                                        .changed()
                                        {
                                            display.color = [
                                                color.r() as f32 / 255.0,
                                                color.g() as f32 / 255.0,
                                                color.b() as f32 / 255.0,
                                                color.a() as f32 / 255.0,
                                            ];
                                        }
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Length:");
                                        ui.add(egui::Slider::new(
                                            &mut display.arrow_length,
                                            0.1..=5.0,
                                        ));
                                    });
                                    ui.horizontal(|ui| {
                                        ui.label("Width:");
                                        ui.add(egui::Slider::new(
                                            &mut display.arrow_width,
                                            0.01..=0.5,
                                        ));
                                    });
                                }
                                _ => {}
                            }

                            if ui.button("Remove").clicked() {
                                to_remove = Some(i);
                            }
                        });

                display.expanded = header.fully_open();
            }
        });

        // Handle removal outside the lock
        if let Some(i) = to_remove {
            self.app_state.lock().remove_display(i);
        }

        ui.separator();

        // Show available topics for adding
        let topics: Vec<_> = {
            let state = self.app_state.lock();
            state.topics.clone()
        };

        ui.menu_button("+ Add Display", |ui| {
            if topics.is_empty() {
                ui.label("No topics available.");
                ui.label("Connect to server first.");
            } else {
                for topic in &topics {
                    if ui.button(&topic.name).clicked() {
                        self.app_state
                            .lock()
                            .add_display(&topic.name, &topic.msg_type);
                        ui.close_menu();
                    }
                }
            }
        });
    }
}
