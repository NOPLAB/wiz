use crate::panels::Panel;
use wiz_core::TopicInfo;

pub struct TopicsPanel {
    filter: String,
    topics: Vec<TopicInfo>,
    selected: Option<usize>,
}

impl TopicsPanel {
    pub fn new() -> Self {
        Self {
            filter: String::new(),
            topics: vec![
                TopicInfo {
                    name: "/velodyne_points".to_string(),
                    msg_type: "sensor_msgs/PointCloud2".to_string(),
                },
                TopicInfo {
                    name: "/scan".to_string(),
                    msg_type: "sensor_msgs/LaserScan".to_string(),
                },
                TopicInfo {
                    name: "/camera/image".to_string(),
                    msg_type: "sensor_msgs/Image".to_string(),
                },
                TopicInfo {
                    name: "/pose".to_string(),
                    msg_type: "geometry_msgs/PoseStamped".to_string(),
                },
                TopicInfo {
                    name: "/path".to_string(),
                    msg_type: "nav_msgs/Path".to_string(),
                },
            ],
            selected: None,
        }
    }

    #[allow(dead_code)]
    pub fn set_topics(&mut self, topics: Vec<TopicInfo>) {
        self.topics = topics;
    }
}

impl Panel for TopicsPanel {
    fn name(&self) -> &str {
        "Topics"
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("\u{1F50D}");
            ui.add(
                egui::TextEdit::singleline(&mut self.filter)
                    .hint_text("Filter...")
                    .desired_width(ui.available_width()),
            );
        });

        ui.separator();

        egui::ScrollArea::vertical().show(ui, |ui| {
            let filter_lower = self.filter.to_lowercase();

            for (i, topic) in self.topics.iter().enumerate() {
                if !self.filter.is_empty()
                    && !topic.name.to_lowercase().contains(&filter_lower)
                    && !topic.msg_type.to_lowercase().contains(&filter_lower)
                {
                    continue;
                }

                let is_selected = self.selected == Some(i);
                let response = ui.selectable_label(is_selected, topic.name.to_string());

                if response.clicked() {
                    self.selected = Some(i);
                }

                if response.double_clicked() {
                    // TODO: Add to displays
                }

                ui.label(
                    egui::RichText::new(&topic.msg_type)
                        .small()
                        .color(egui::Color32::GRAY),
                );

                ui.add_space(4.0);
            }
        });

        ui.separator();

        ui.horizontal(|ui| {
            if ui.button("+ Add Selected").clicked() {
                // TODO: Add selected topic to displays
            }
            if ui.button("\u{21BB} Refresh").clicked() {
                // TODO: Refresh topic list
            }
        });
    }
}
