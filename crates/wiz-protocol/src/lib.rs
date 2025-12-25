pub mod codec;

pub use codec::*;

use serde::{Deserialize, Serialize};
use wiz_core::{TopicInfo, Transform3D};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ClientMessage {
    #[serde(rename = "subscribe")]
    Subscribe {
        id: String,
        topic: String,
        msg_type: String,
        #[serde(default)]
        throttle_rate: Option<u32>,
    },
    #[serde(rename = "unsubscribe")]
    Unsubscribe { id: String },
    #[serde(rename = "tf_lookup")]
    TfLookup {
        target_frame: String,
        source_frame: String,
    },
    #[serde(rename = "list_topics")]
    ListTopics,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ServerMessage {
    #[serde(rename = "data")]
    Data {
        topic: String,
        msg_type: String,
        timestamp: f64,
        #[serde(with = "serde_bytes")]
        payload: Vec<u8>,
    },
    #[serde(rename = "tf")]
    Transform {
        target_frame: String,
        source_frame: String,
        transform: Transform3D,
    },
    #[serde(rename = "topics")]
    Topics { topics: Vec<TopicInfo> },
    #[serde(rename = "subscribed")]
    Subscribed { id: String, topic: String },
    #[serde(rename = "unsubscribed")]
    Unsubscribed { id: String },
    #[serde(rename = "error")]
    Error { message: String },
}
