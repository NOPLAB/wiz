use crate::app_state::SharedAppState;
use crate::panels::Panel;

pub struct TopicsPanel {
    app_state: SharedAppState,
}

impl TopicsPanel {
    pub fn new(app_state: SharedAppState) -> Self {
        Self { app_state }
    }
}

impl Panel for TopicsPanel {
    fn name(&self) -> &str {
        "Topics"
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        // Filter input
        {
            let mut state = self.app_state.lock();
            ui.horizontal(|ui| {
                ui.label("\u{1F50D}");
                ui.add(
                    egui::TextEdit::singleline(&mut state.topic_filter)
                        .hint_text("Filter...")
                        .desired_width(ui.available_width()),
                );
            });
        }

        ui.separator();

        // Topic list
        let mut add_selected = false;
        let mut refresh = false;
        let mut clicked_index: Option<usize> = None;
        let mut double_clicked = false;
        let mut toggle_action: Option<(String, String, bool)> = None; // (topic, msg_type, new_visible)

        egui::ScrollArea::vertical().show(ui, |ui| {
            let state = self.app_state.lock();
            let filter_lower = state.topic_filter.to_lowercase();

            if state.topics.is_empty() {
                ui.label("No topics available.");
                ui.label("Connect to server to see topics.");
            }

            for (i, topic) in state.topics.iter().enumerate() {
                if !state.topic_filter.is_empty()
                    && !topic.name.to_lowercase().contains(&filter_lower)
                    && !topic.msg_type.to_lowercase().contains(&filter_lower)
                {
                    continue;
                }

                // Check if this topic has a display and its visibility
                // Default to visible (checked) if no display exists yet
                let display_info = state.displays.iter().find(|d| d.topic == topic.name);
                let is_visible = display_info.map(|d| d.visible).unwrap_or(true);
                let is_selected = state.selected_topic == Some(i);

                ui.horizontal(|ui| {
                    // Checkbox for 3D visibility toggle
                    let mut checked = is_visible;
                    if ui.checkbox(&mut checked, "").changed() {
                        toggle_action = Some((topic.name.clone(), topic.msg_type.clone(), checked));
                    }

                    // Selectable topic name
                    let response = ui.selectable_label(is_selected, &topic.name);
                    if response.clicked() {
                        clicked_index = Some(i);
                    }
                    if response.double_clicked() {
                        clicked_index = Some(i);
                        double_clicked = true;
                    }
                });

                ui.label(
                    egui::RichText::new(&topic.msg_type)
                        .small()
                        .color(egui::Color32::GRAY),
                );

                ui.add_space(4.0);
            }
        });

        // Handle selection changes outside the lock
        if let Some(idx) = clicked_index {
            self.app_state.lock().selected_topic = Some(idx);
        }
        if double_clicked {
            self.app_state.lock().add_selected_topic_as_display();
        }

        // Handle visibility toggle
        if let Some((topic, msg_type, new_visible)) = toggle_action {
            let mut state = self.app_state.lock();
            if let Some(display) = state.displays.iter_mut().find(|d| d.topic == topic) {
                // Display exists, toggle visibility
                display.visible = new_visible;
            } else if new_visible {
                // No display yet, add one with visible=true
                state.add_display(&topic, &msg_type);
            }
        }

        ui.separator();

        ui.horizontal(|ui| {
            if ui.button("+ Add Selected").clicked() {
                add_selected = true;
            }
            if ui.button("\u{21BB} Refresh").clicked() {
                refresh = true;
            }
        });

        // Handle button actions outside the scroll area
        if add_selected {
            self.app_state.lock().add_selected_topic_as_display();
        }
        if refresh {
            self.app_state.lock().request_refresh();
        }
    }
}
