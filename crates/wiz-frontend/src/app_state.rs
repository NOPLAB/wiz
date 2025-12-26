use parking_lot::Mutex;
use std::collections::VecDeque;
use std::sync::Arc;
use wiz_core::TopicInfo;

use crate::panels::property::{Display, DisplayType};

/// Shared application state accessible by all panels
pub type SharedAppState = Arc<Mutex<AppState>>;

/// Performance statistics for the Performance panel
#[derive(Default)]
pub struct PerformanceStats {
    /// Bytes received in the last second
    pub bytes_received: u64,
    /// Messages received in the last second
    pub messages_received: u32,
    /// Estimated latency in milliseconds (smoothed)
    pub latency_ms: f32,
    /// Bandwidth in MB/s
    pub bandwidth_mbs: f32,
    /// GPU memory usage in MB (estimated from buffer sizes)
    pub gpu_memory_mb: f32,
    /// CPU memory usage in MB
    pub cpu_memory_mb: f32,
    /// Point count from renderer
    pub point_count: usize,
    /// Triangle count from renderer
    pub triangle_count: usize,
    /// History of bytes received per update (for bandwidth calculation)
    bytes_history: VecDeque<u64>,
    /// History of message counts per update
    message_history: VecDeque<u32>,
    /// History of latency measurements for smoothing
    latency_history: VecDeque<f32>,
    /// Total GPU buffer bytes (for GPU memory estimation)
    gpu_buffer_bytes: u64,
    /// Total messages ever received (to check if data has flowed)
    pub total_messages_received: u64,
}

impl PerformanceStats {
    pub fn new() -> Self {
        Self {
            bytes_history: VecDeque::with_capacity(60),
            message_history: VecDeque::with_capacity(60),
            latency_history: VecDeque::with_capacity(30),
            ..Default::default()
        }
    }

    /// Record received bytes and message count
    pub fn record_data(&mut self, bytes: u64, messages: u32) {
        self.bytes_received += bytes;
        self.messages_received += messages;
        self.total_messages_received += messages as u64;
    }

    /// Check if any data has ever been received
    pub fn has_received_data(&self) -> bool {
        self.total_messages_received > 0
    }

    /// Record latency from server timestamp (in seconds since epoch)
    pub fn record_latency(&mut self, server_timestamp: f64) {
        let now = web_time::SystemTime::now()
            .duration_since(web_time::SystemTime::UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);

        // Calculate latency (can be negative if clocks are out of sync)
        let latency = (now - server_timestamp) * 1000.0; // Convert to ms

        // Only record reasonable latency values (0-5000ms)
        if (0.0..5000.0).contains(&latency) {
            self.latency_history.push_back(latency as f32);
            if self.latency_history.len() > 30 {
                self.latency_history.pop_front();
            }
        }
    }

    /// Update GPU memory estimate based on point cloud size
    pub fn update_gpu_memory(&mut self, point_count: usize, bytes_per_point: usize) {
        // Estimate: position (12 bytes) + color (4 bytes) = 16 bytes per point minimum
        // Plus index buffers, uniform buffers, etc.
        let point_buffer_bytes = point_count * bytes_per_point;
        self.gpu_buffer_bytes = point_buffer_bytes as u64;
        self.gpu_memory_mb = self.gpu_buffer_bytes as f32 / 1_000_000.0;
        self.point_count = point_count;
    }

    /// Update stats (called once per frame or periodically)
    pub fn update(&mut self) {
        // Push current values to history
        self.bytes_history.push_back(self.bytes_received);
        self.message_history.push_back(self.messages_received);

        // Keep only last 60 samples
        if self.bytes_history.len() > 60 {
            self.bytes_history.pop_front();
        }
        if self.message_history.len() > 60 {
            self.message_history.pop_front();
        }

        // Calculate bandwidth (average over last second, assuming ~60 fps)
        let total_bytes: u64 = self.bytes_history.iter().sum();
        self.bandwidth_mbs = total_bytes as f32 / 1_000_000.0;

        // Calculate smoothed latency
        if !self.latency_history.is_empty() {
            self.latency_ms =
                self.latency_history.iter().sum::<f32>() / self.latency_history.len() as f32;
        }

        // Update CPU memory
        self.cpu_memory_mb = get_cpu_memory_mb();

        // Reset counters for next frame
        self.bytes_received = 0;
        self.messages_received = 0;
    }

    /// Get messages per second
    pub fn messages_per_sec(&self) -> u32 {
        self.message_history.iter().sum()
    }
}

/// Get CPU memory usage in MB (platform-specific)
#[cfg(target_arch = "wasm32")]
fn get_cpu_memory_mb() -> f32 {
    // On WASM, get the memory size from the WebAssembly memory
    // Each page is 64KB
    let pages = core::arch::wasm32::memory_size(0);
    (pages * 65536) as f32 / 1_000_000.0
}

#[cfg(not(target_arch = "wasm32"))]
fn get_cpu_memory_mb() -> f32 {
    // On native platforms, try to read from /proc/self/status on Linux
    #[cfg(target_os = "linux")]
    {
        if let Ok(status) = std::fs::read_to_string("/proc/self/status") {
            for line in status.lines() {
                if line.starts_with("VmRSS:") {
                    // VmRSS is in KB
                    if let Some(kb_str) = line.split_whitespace().nth(1) {
                        if let Ok(kb) = kb_str.parse::<u64>() {
                            return kb as f32 / 1000.0; // Convert to MB
                        }
                    }
                }
            }
        }
    }

    // Fallback: estimate from allocator (rough estimate)
    // This is a simple heuristic based on typical Rust program memory usage
    0.0
}

/// Application state shared between panels
pub struct AppState {
    /// Available topics from the server
    pub topics: Vec<TopicInfo>,
    /// Active displays
    pub displays: Vec<Display>,
    /// Selected topic index in the topics panel
    pub selected_topic: Option<usize>,
    /// Topic filter string
    pub topic_filter: String,
    /// Next subscription ID
    next_subscription_id: u32,
    /// Pending actions to be processed by the main app
    pub pending_actions: Vec<AppAction>,
    /// Performance statistics
    pub performance_stats: PerformanceStats,
}

/// Actions that panels can request
#[derive(Debug, Clone)]
pub enum AppAction {
    /// Subscribe to a topic
    Subscribe { topic: String, msg_type: String },
    /// Unsubscribe from a topic
    Unsubscribe { subscription_id: u32 },
    /// Refresh the topic list
    RefreshTopics,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            topics: Vec::new(),
            displays: Vec::new(),
            selected_topic: None,
            topic_filter: String::new(),
            next_subscription_id: 1,
            pending_actions: Vec::new(),
            performance_stats: PerformanceStats::new(),
        }
    }

    /// Update the topic list from the server
    pub fn set_topics(&mut self, topics: Vec<TopicInfo>) {
        self.topics = topics;
        // Reset selection if out of bounds
        if let Some(idx) = self.selected_topic {
            if idx >= self.topics.len() {
                self.selected_topic = None;
            }
        }
    }

    /// Add a display for a topic
    pub fn add_display(&mut self, topic: &str, msg_type: &str) -> u32 {
        let display_type = match msg_type {
            s if s.contains("PointCloud2") => DisplayType::PointCloud2,
            s if s.contains("LaserScan") => DisplayType::LaserScan,
            s if s.contains("PoseStamped") => DisplayType::Pose,
            s if s.contains("Path") => DisplayType::Path,
            s if s.contains("Marker") => DisplayType::Marker,
            s if s.contains("TFMessage") => DisplayType::TF,
            s if s.contains("Odometry") => DisplayType::Odometry,
            s if s.contains("Imu") => DisplayType::Imu,
            s if s.contains("Twist") => DisplayType::Twist,
            s if s.contains("JointState") => DisplayType::JointState,
            s if s.contains("Image") => DisplayType::Image,
            s if s.contains("CameraInfo") => DisplayType::CameraInfo,
            s if s.contains("Clock") => DisplayType::Clock,
            s if s.contains("Log") => DisplayType::Log,
            s if s.contains("String") => DisplayType::String,
            _ => DisplayType::PointCloud2, // Default
        };

        let subscription_id = self.next_subscription_id;
        self.next_subscription_id += 1;

        let display = Display::new(display_type, topic.to_string(), subscription_id);
        self.displays.push(display);

        // Queue subscribe action
        self.pending_actions.push(AppAction::Subscribe {
            topic: topic.to_string(),
            msg_type: msg_type.to_string(),
        });

        subscription_id
    }

    /// Remove a display by index
    pub fn remove_display(&mut self, index: usize) {
        if index < self.displays.len() {
            let display = self.displays.remove(index);
            self.pending_actions.push(AppAction::Unsubscribe {
                subscription_id: display.subscription_id,
            });
        }
    }

    /// Get the currently selected topic info
    pub fn get_selected_topic(&self) -> Option<&TopicInfo> {
        self.selected_topic.and_then(|idx| self.topics.get(idx))
    }

    /// Add the currently selected topic as a display
    pub fn add_selected_topic_as_display(&mut self) -> Option<u32> {
        if let Some(topic) = self.get_selected_topic().cloned() {
            Some(self.add_display(&topic.name, &topic.msg_type))
        } else {
            None
        }
    }

    /// Request a topic list refresh
    pub fn request_refresh(&mut self) {
        self.pending_actions.push(AppAction::RefreshTopics);
    }

    /// Take all pending actions (clears the list)
    pub fn take_pending_actions(&mut self) -> Vec<AppAction> {
        std::mem::take(&mut self.pending_actions)
    }

    /// Update display text data for a topic
    pub fn update_display_text(&mut self, topic: &str, text: String, log_level: Option<String>) {
        for display in &mut self.displays {
            if display.topic == topic {
                display.text_data = Some(text.clone());
                display.log_level = log_level.clone();
            }
        }
    }
}

impl Default for AppState {
    fn default() -> Self {
        Self::new()
    }
}
