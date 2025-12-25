use crate::panels::Panel;
use std::collections::VecDeque;

pub struct PerformancePanel {
    fps_history: VecDeque<f32>,
    last_frame_time: std::time::Instant,
    frame_times: VecDeque<f32>,
    point_count: usize,
    triangle_count: usize,
    latency_ms: f32,
    bandwidth_mbs: f32,
    messages_per_sec: u32,
    gpu_memory_mb: f32,
    cpu_memory_mb: f32,
}

impl PerformancePanel {
    pub fn new() -> Self {
        Self {
            fps_history: VecDeque::with_capacity(120),
            last_frame_time: std::time::Instant::now(),
            frame_times: VecDeque::with_capacity(60),
            point_count: 0,
            triangle_count: 0,
            latency_ms: 0.0,
            bandwidth_mbs: 0.0,
            messages_per_sec: 0,
            gpu_memory_mb: 0.0,
            cpu_memory_mb: 0.0,
        }
    }

    pub fn update(&mut self) {
        let now = std::time::Instant::now();
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

                ui.add_space(8.0);
                ui.end_row();

                ui.label(egui::RichText::new("Data").strong());
                ui.end_row();

                ui.label("Points:");
                ui.label(format_number(self.point_count));
                ui.end_row();

                ui.label("Triangles:");
                ui.label(format_number(self.triangle_count));
                ui.end_row();

                ui.add_space(8.0);
                ui.end_row();

                ui.label(egui::RichText::new("Network").strong());
                ui.end_row();

                ui.label("Latency:");
                ui.label(format!("{:.0} ms", self.latency_ms));
                ui.end_row();

                ui.label("Bandwidth:");
                ui.label(format!("{:.1} MB/s", self.bandwidth_mbs));
                ui.end_row();

                ui.label("Messages:");
                ui.label(format!("{}/s", self.messages_per_sec));
                ui.end_row();

                ui.add_space(8.0);
                ui.end_row();

                ui.label(egui::RichText::new("Memory").strong());
                ui.end_row();

                ui.label("GPU:");
                ui.label(format!("{:.0} MB", self.gpu_memory_mb));
                ui.end_row();

                ui.label("CPU:");
                ui.label(format!("{:.0} MB", self.cpu_memory_mb));
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
