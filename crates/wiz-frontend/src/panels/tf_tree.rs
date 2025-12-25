use crate::panels::Panel;

#[derive(Debug, Clone)]
struct TfNode {
    name: String,
    children: Vec<TfNode>,
}

impl TfNode {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            children: Vec::new(),
        }
    }

    fn with_children(name: &str, children: Vec<TfNode>) -> Self {
        Self {
            name: name.to_string(),
            children,
        }
    }
}

pub struct TfTreePanel {
    root: TfNode,
    selected_frame: Option<String>,
    fixed_frame: String,
    filter: String,
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
                        TfNode::with_children(
                            "wheel_left",
                            vec![],
                        ),
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
        }
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
        ui.horizontal(|ui| {
            ui.label("\u{1F50D}");
            ui.add(
                egui::TextEdit::singleline(&mut self.filter)
                    .hint_text("Filter...")
                    .desired_width(100.0),
            );

            ui.label("Fixed:");
            egui::ComboBox::from_id_salt("tf_fixed_frame")
                .selected_text(&self.fixed_frame)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.fixed_frame, "map".to_string(), "map");
                    ui.selectable_value(&mut self.fixed_frame, "odom".to_string(), "odom");
                    ui.selectable_value(&mut self.fixed_frame, "base_link".to_string(), "base_link");
                });
        });

        ui.separator();

        egui::ScrollArea::vertical().show(ui, |ui| {
            let root = self.root.clone();
            self.render_node(ui, &root, 0);
        });

        ui.separator();

        if let Some(ref frame) = self.selected_frame {
            ui.label(format!("Selected: {}", frame));
            // TODO: Show position/rotation info
        }
    }
}
