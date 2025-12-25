use crate::panels::Panel;

#[derive(Debug, Clone)]
struct TfNode {
    name: String,
    children: Vec<TfNode>,
    #[allow(dead_code)]
    visible: bool,
}

impl TfNode {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            children: Vec::new(),
            visible: true,
        }
    }

    fn with_children(name: &str, children: Vec<TfNode>) -> Self {
        Self {
            name: name.to_string(),
            children,
            visible: true,
        }
    }
}

pub struct TfTreePanel {
    root: TfNode,
    selected_frame: Option<String>,
    fixed_frame: String,
    filter: String,
    show_all_frames: bool,
    axis_length: f32,
}

impl TfTreePanel {
    pub fn new() -> Self {
        // Example TF tree
        let root = TfNode::with_children(
            "map",
            vec![TfNode::with_children(
                "odom",
                vec![TfNode::with_children(
                    "base_link",
                    vec![
                        TfNode::new("laser_frame"),
                        TfNode::new("camera_link"),
                        TfNode::with_children("wheel_left", vec![]),
                        TfNode::new("wheel_right"),
                    ],
                )],
            )],
        );

        Self {
            root,
            selected_frame: None,
            fixed_frame: "map".to_string(),
            filter: String::new(),
            show_all_frames: true,
            axis_length: 0.5,
        }
    }

    /// Get the axis length setting
    #[allow(dead_code)]
    pub fn axis_length(&self) -> f32 {
        self.axis_length
    }

    /// Check if all frames should be shown
    #[allow(dead_code)]
    pub fn show_all_frames(&self) -> bool {
        self.show_all_frames
    }

    fn render_node(&mut self, ui: &mut egui::Ui, node: &TfNode, depth: usize) {
        let indent = depth as f32 * 16.0;
        ui.horizontal(|ui| {
            ui.add_space(indent);

            let is_selected = self.selected_frame.as_ref() == Some(&node.name);
            let label = if node.children.is_empty() {
                format!("\u{2514}\u{2500} {}", node.name)
            } else {
                format!("\u{251C}\u{2500}\u{252C} {}", node.name)
            };

            let response = ui.selectable_label(is_selected, label);
            if response.clicked() {
                self.selected_frame = Some(node.name.clone());
            }
            if response.double_clicked() {
                // TODO: Focus camera on this frame
            }
            response.context_menu(|ui| {
                if ui.button("Set as Fixed Frame").clicked() {
                    self.fixed_frame = node.name.clone();
                    ui.close_menu();
                }
                if ui.button("Focus Camera").clicked() {
                    // TODO: Focus camera
                    ui.close_menu();
                }
            });
        });

        for child in &node.children {
            self.render_node(ui, child, depth + 1);
        }
    }
}

impl Panel for TfTreePanel {
    fn name(&self) -> &str {
        "TF Tree"
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        // Settings section
        egui::CollapsingHeader::new("Settings")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.show_all_frames, "Show Axes");
                });

                ui.horizontal(|ui| {
                    ui.label("Axis Length:");
                    ui.add(
                        egui::Slider::new(&mut self.axis_length, 0.1..=2.0)
                            .step_by(0.1)
                            .suffix("m"),
                    );
                });

                ui.horizontal(|ui| {
                    ui.label("Fixed:");
                    egui::ComboBox::from_id_salt("tf_fixed_frame")
                        .selected_text(&self.fixed_frame)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.fixed_frame, "map".to_string(), "map");
                            ui.selectable_value(&mut self.fixed_frame, "odom".to_string(), "odom");
                            ui.selectable_value(
                                &mut self.fixed_frame,
                                "base_link".to_string(),
                                "base_link",
                            );
                        });
                });
            });

        ui.separator();

        // Filter and tree
        ui.horizontal(|ui| {
            ui.label("Filter:");
            ui.add(
                egui::TextEdit::singleline(&mut self.filter)
                    .hint_text("frame name...")
                    .desired_width(120.0),
            );
        });

        ui.separator();

        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                let root = self.root.clone();
                self.render_node(ui, &root, 0);
            });

        ui.separator();

        // Selected frame info
        if let Some(ref frame) = self.selected_frame {
            egui::CollapsingHeader::new(format!("Frame: {frame}"))
                .default_open(true)
                .show(ui, |ui| {
                    ui.label("Position:");
                    ui.horizontal(|ui| {
                        ui.label("  X: 0.000");
                        ui.label("  Y: 0.000");
                        ui.label("  Z: 0.000");
                    });
                    ui.label("Rotation (RPY):");
                    ui.horizontal(|ui| {
                        ui.label("  R: 0.000");
                        ui.label("  P: 0.000");
                        ui.label("  Y: 0.000");
                    });
                });
        }
    }
}
