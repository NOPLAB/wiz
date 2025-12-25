use parking_lot::RwLock;
use std::{collections::HashMap, sync::Arc};
use tokio::sync::broadcast;
use wiz_core::TopicInfo;
use wiz_protocol::ServerMessage;

#[derive(Clone)]
pub struct AppState {
    inner: Arc<AppStateInner>,
}

struct AppStateInner {
    subscriptions: RwLock<HashMap<String, SubscriptionInfo>>,
    tx: broadcast::Sender<ServerMessage>,
}

#[allow(dead_code)]
struct SubscriptionInfo {
    topic: String,
    msg_type: String,
    throttle_rate: Option<u32>,
}

impl AppState {
    pub fn new() -> Self {
        let (tx, _) = broadcast::channel(1024);
        Self {
            inner: Arc::new(AppStateInner {
                subscriptions: RwLock::new(HashMap::new()),
                tx,
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
        subs.remove(id).map(|s| s.topic)
    }

    pub fn list_topics(&self) -> Vec<TopicInfo> {
        // Mock topics for now - will be replaced with actual ROS2 bridge
        vec![
            TopicInfo {
                name: "/velodyne_points".to_string(),
                msg_type: "sensor_msgs/msg/PointCloud2".to_string(),
            },
            TopicInfo {
                name: "/scan".to_string(),
                msg_type: "sensor_msgs/msg/LaserScan".to_string(),
            },
            TopicInfo {
                name: "/tf".to_string(),
                msg_type: "tf2_msgs/msg/TFMessage".to_string(),
            },
            TopicInfo {
                name: "/robot_pose".to_string(),
                msg_type: "geometry_msgs/msg/PoseStamped".to_string(),
            },
        ]
    }
}
