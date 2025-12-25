use parking_lot::Mutex;
use std::sync::Arc;
use wiz_core::TopicInfo;

use crate::panels::displays::{Display, DisplayType};

/// Shared application state accessible by all panels
pub type SharedAppState = Arc<Mutex<AppState>>;

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
}

impl Default for AppState {
    fn default() -> Self {
        Self::new()
    }
}
