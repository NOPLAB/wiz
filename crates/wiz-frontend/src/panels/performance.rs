use crate::app_state::SharedAppState;
use crate::panels::Panel;
use std::collections::VecDeque;
use web_time::Instant;

pub struct PerformancePanel {
    app_state: SharedAppState,
    fps_history: VecDeque<f32>,
    last_frame_time: Instant,
    frame_times: VecDeque<f32>,
}

impl PerformancePanel {
    pub fn new(app_state: SharedAppState) -> Self {
        Self {
            app_state,
            fps_history: VecDeque::with_capacity(120),
            last_frame_time: Instant::now(),
            frame_times: VecDeque::with_capacity(60),
        }
    }

    pub fn update(&mut self) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_frame_time).as_secs_f32();
        self.last_frame_time = now;

        let fps = if dt > 0.0 { 1.0 / dt } else { 0.0 };
        self.fps_history.push_back(fps);
        if self.fps_history.len() > 120 {
            self.fps_history.pop_front();
        }

        self.frame_times.push_back(dt * 1000.0);
        if self.frame_times.len() > 60 {
            self.frame_times.pop_front();
        }
    }

    fn average_fps(&self) -> f32 {
        if self.fps_history.is_empty() {
            return 0.0;
        }
        self.fps_history.iter().sum::<f32>() / self.fps_history.len() as f32
    }

    fn average_frame_time(&self) -> f32 {
        if self.frame_times.is_empty() {
            return 0.0;
        }
        self.frame_times.iter().sum::<f32>() / self.frame_times.len() as f32
    }
}

impl Panel for PerformancePanel {
    fn name(&self) -> &str {
        "Performance"
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        self.update();

        // Get stats from shared state
        let state = self.app_state.lock();
        let stats = &state.performance_stats;
        let latency_ms = stats.latency_ms;
        let bandwidth_mbs = stats.bandwidth_mbs;
        let messages_per_sec = stats.messages_per_sec();
        let gpu_memory_mb = stats.gpu_memory_mb;
        let cpu_memory_mb = stats.cpu_memory_mb;
        let point_count = stats.point_count;
        let triangle_count = stats.triangle_count;
        let has_data = stats.has_received_data();
        let total_msgs = stats.total_messages_received;
        drop(state);

        egui::Grid::new("performance_grid")
            .num_columns(2)
            .spacing([20.0, 4.0])
            .show(ui, |ui| {
                ui.label(egui::RichText::new("Rendering").strong());
                ui.end_row();

                ui.label("FPS:");
                ui.label(format!("{:.1}", self.average_fps()));
                ui.end_row();

                ui.label("Frame Time:");
                ui.label(format!("{:.2} ms", self.average_frame_time()));
                ui.end_row();

                ui.label("Draw Calls:");
                ui.label("12");
                ui.end_row();

                ui.label("");
                ui.end_row();

                ui.label(egui::RichText::new("Data").strong());
                ui.end_row();

                ui.label("Points:");
                ui.label(format_number(point_count));
                ui.end_row();

                ui.label("Triangles:");
                ui.label(format_number(triangle_count));
                ui.end_row();

                ui.label("");
                ui.end_row();

                ui.label(egui::RichText::new("Network").strong());
                ui.end_row();

                ui.label("Latency:");
                if has_data && latency_ms > 0.0 {
                    ui.label(format!("{latency_ms:.0} ms"));
                } else if has_data {
                    ui.label(egui::RichText::new("< 1 ms").weak());
                } else {
                    ui.label(egui::RichText::new("No data").weak());
                }
                ui.end_row();

                ui.label("Bandwidth:");
                if has_data {
                    ui.label(format!("{bandwidth_mbs:.2} MB/s"));
                } else {
                    ui.label(egui::RichText::new("-").weak());
                }
                ui.end_row();

                ui.label("Messages:");
                if has_data {
                    ui.label(format!("{messages_per_sec}/s"));
                } else {
                    ui.label(egui::RichText::new("-").weak());
                }
                ui.end_row();

                ui.label("Total:");
                ui.label(format_number(total_msgs as usize));
                ui.end_row();

                ui.label("");
                ui.end_row();

                ui.label(egui::RichText::new("Memory").strong());
                ui.end_row();

                ui.label("GPU:");
                ui.label(format!("{gpu_memory_mb:.1} MB"));
                ui.end_row();

                ui.label("CPU:");
                ui.label(format!("{cpu_memory_mb:.1} MB"));
                ui.end_row();
            });

        ui.separator();

        // FPS graph (simplified - bar visualization)
        ui.collapsing("FPS Graph", |ui| {
            let fps_data: Vec<f32> = self.fps_history.iter().copied().collect();
            let max_fps = fps_data.iter().cloned().fold(60.0_f32, f32::max);

            let (response, painter) =
                ui.allocate_painter(egui::vec2(ui.available_width(), 60.0), egui::Sense::hover());

            let rect = response.rect;
            painter.rect_filled(rect, 0.0, egui::Color32::from_gray(40));

            if !fps_data.is_empty() {
                let bar_width = rect.width() / fps_data.len() as f32;
                for (i, &fps) in fps_data.iter().enumerate() {
                    let height = (fps / max_fps) * rect.height();
                    let x = rect.left() + i as f32 * bar_width;
                    let bar_rect = egui::Rect::from_min_max(
                        egui::pos2(x, rect.bottom() - height),
                        egui::pos2(x + bar_width - 1.0, rect.bottom()),
                    );
                    let color = if fps >= 55.0 {
                        egui::Color32::GREEN
                    } else if fps >= 30.0 {
                        egui::Color32::YELLOW
                    } else {
                        egui::Color32::RED
                    };
                    painter.rect_filled(bar_rect, 0.0, color);
                }
            }
        });
    }
}

fn format_number(n: usize) -> String {
    if n >= 1_000_000 {
        format!("{:.2}M", n as f64 / 1_000_000.0)
    } else if n >= 1_000 {
        format!("{:.1}K", n as f64 / 1_000.0)
    } else {
        n.to_string()
    }
}
