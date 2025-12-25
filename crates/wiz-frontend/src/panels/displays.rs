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
}

impl Display {
    pub fn new(display_type: DisplayType, topic: String) -> Self {
        Self {
            display_type,
            topic,
            visible: true,
            expanded: true,
            point_size: 2.0,
            alpha: 1.0,
            color: [1.0, 0.0, 0.0, 1.0],
        }
    }
}

pub struct DisplaysPanel {
    displays: Vec<Display>,
}

impl DisplaysPanel {
    pub fn new() -> Self {
        Self {
            displays: vec![
                Display::new(DisplayType::PointCloud2, "/velodyne_points".to_string()),
                Display::new(DisplayType::LaserScan, "/scan".to_string()),
            ],
        }
    }

    #[allow(dead_code)]
    pub fn add_display(&mut self, display: Display) {
        self.displays.push(display);
    }
}

impl Panel for DisplaysPanel {
    fn name(&self) -> &str {
        "Displays"
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            let mut to_remove = None;

            for (i, display) in self.displays.iter_mut().enumerate() {
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
                                _ => {}
                            }

                            if ui.button("Remove").clicked() {
                                to_remove = Some(i);
                            }
                        });

                display.expanded = header.fully_open();
            }

            if let Some(i) = to_remove {
                self.displays.remove(i);
            }
        });

        ui.separator();

        ui.menu_button("+ Add Display", |ui| {
            if ui.button("PointCloud2").clicked() {
                self.displays
                    .push(Display::new(DisplayType::PointCloud2, "/topic".to_string()));
                ui.close_menu();
            }
            if ui.button("LaserScan").clicked() {
                self.displays
                    .push(Display::new(DisplayType::LaserScan, "/topic".to_string()));
                ui.close_menu();
            }
            if ui.button("TF").clicked() {
                self.displays
                    .push(Display::new(DisplayType::TF, "".to_string()));
                ui.close_menu();
            }
            if ui.button("Marker").clicked() {
                self.displays
                    .push(Display::new(DisplayType::Marker, "/topic".to_string()));
                ui.close_menu();
            }
        });
    }
}
