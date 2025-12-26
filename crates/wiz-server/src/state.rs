use parking_lot::RwLock;
use std::{collections::HashMap, sync::Arc};
use tokio::sync::broadcast;
use wiz_core::TopicInfo;
use wiz_protocol::ServerMessage;

#[cfg(feature = "ros2")]
use crate::bridge::Ros2Bridge;

#[derive(Clone)]
pub struct AppState {
    inner: Arc<AppStateInner>,
}

struct AppStateInner {
    subscriptions: RwLock<HashMap<String, SubscriptionInfo>>,
    tx: broadcast::Sender<ServerMessage>,
    #[cfg(feature = "ros2")]
    ros2_bridge: Option<Ros2Bridge>,
}

#[allow(dead_code)]
struct SubscriptionInfo {
    topic: String,
    msg_type: String,
    throttle_rate: Option<u32>,
}

impl Default for AppState {
    fn default() -> Self {
        Self::new()
    }
}

impl AppState {
    pub fn new() -> Self {
        let (tx, _) = broadcast::channel(1024);
        Self {
            inner: Arc::new(AppStateInner {
                subscriptions: RwLock::new(HashMap::new()),
                tx,
                #[cfg(feature = "ros2")]
                ros2_bridge: None,
            }),
        }
    }

    #[cfg(feature = "ros2")]
    pub fn with_ros2_bridge(ros2_bridge: Ros2Bridge) -> Self {
        let (tx, _) = broadcast::channel(1024);
        Self {
            inner: Arc::new(AppStateInner {
                subscriptions: RwLock::new(HashMap::new()),
                tx,
                ros2_bridge: Some(ros2_bridge),
            }),
        }
    }

    pub fn subscribe(&self) -> broadcast::Receiver<ServerMessage> {
        self.inner.tx.subscribe()
    }

    pub fn broadcast(&self, msg: ServerMessage) {
        let _ = self.inner.tx.send(msg);
    }

    pub fn add_subscription(
        &self,
        id: String,
        topic: String,
        msg_type: String,
        throttle_rate: Option<u32>,
    ) {
        // Subscribe to the topic via ROS2 bridge if available
        #[cfg(feature = "ros2")]
        if let Some(ref bridge) = self.inner.ros2_bridge {
            match msg_type.as_str() {
                "sensor_msgs/msg/PointCloud2" => {
                    bridge.subscribe_pointcloud2(&topic);
                }
                "sensor_msgs/msg/LaserScan" => {
                    bridge.subscribe_laserscan(&topic);
                }
                // Other message types can be added here
                _ => {}
            }
        }

        let mut subs = self.inner.subscriptions.write();
        subs.insert(
            id,
            SubscriptionInfo {
                topic,
                msg_type,
                throttle_rate,
            },
        );
    }

    pub fn remove_subscription(&self, id: &str) -> Option<String> {
        let mut subs = self.inner.subscriptions.write();
        if let Some(sub) = subs.remove(id) {
            // Unsubscribe from the topic via ROS2 bridge if available
            #[cfg(feature = "ros2")]
            if let Some(ref bridge) = self.inner.ros2_bridge {
                bridge.unsubscribe(&sub.topic);
            }
            Some(sub.topic)
        } else {
            None
        }
    }

    /// Get all active subscriptions as (topic, msg_type) pairs
    pub fn get_subscriptions(&self) -> Vec<(String, String)> {
        let subs = self.inner.subscriptions.read();
        subs.values()
            .map(|s| (s.topic.clone(), s.msg_type.clone()))
            .collect()
    }

    pub fn list_topics(&self) -> Vec<TopicInfo> {
        #[cfg(feature = "ros2")]
        if let Some(ref bridge) = self.inner.ros2_bridge {
            return bridge.list_topics();
        }

        // Return empty list when ROS2 bridge is not available
        Vec::new()
    }

    /// Get a reference to the ROS2 bridge if available
    #[cfg(feature = "ros2")]
    pub fn ros2_bridge(&self) -> Option<&Ros2Bridge> {
        self.inner.ros2_bridge.as_ref()
    }

    /// Check if ROS2 bridge is available and initialized
    #[cfg(feature = "ros2")]
    pub fn has_ros2_bridge(&self) -> bool {
        self.inner.ros2_bridge.is_some()
    }

    #[cfg(not(feature = "ros2"))]
    pub fn has_ros2_bridge(&self) -> bool {
        false
    }

    /// Spin the ROS2 bridge to process callbacks
    #[cfg(feature = "ros2")]
    pub fn spin_once(&self) {
        if let Some(ref bridge) = self.inner.ros2_bridge {
            bridge.spin_once();
        }
    }
}
