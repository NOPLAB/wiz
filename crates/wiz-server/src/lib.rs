//! wiz-server library
//!
//! This module exports the server components for use in tests and other applications.

mod bridge;
mod ffi;
mod mock_data;
mod state;
mod ws;

use axum::{Router, routing::get};
use tower_http::cors::{Any, CorsLayer};

pub use state::AppState;

/// Create the application router with the given state
pub fn create_app(state: AppState) -> Router {
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    Router::new()
        .route("/ws", get(ws::ws_handler))
        .route("/health", get(|| async { "OK" }))
        .layer(cors)
        .with_state(state)
}

/// Start the mock data generator task
pub fn spawn_mock_data_generator(state: AppState) {
    use std::time::Duration;
    use wiz_protocol::ServerMessage;

    use mock_data::MockDataGenerator;

    let mock_state = state.clone();
    tokio::spawn(async move {
        let mut generator = MockDataGenerator::new();
        let mut interval = tokio::time::interval(Duration::from_millis(100)); // 10 Hz

        loop {
            interval.tick().await;
            generator.tick();

            // Get active subscriptions and send mock data
            let subscriptions = mock_state.get_subscriptions();

            for (topic, msg_type) in subscriptions {
                let payload = match msg_type.as_str() {
                    "sensor_msgs/msg/LaserScan" => {
                        let scan = generator.generate_laser_scan(&topic);
                        rmp_serde::to_vec(&scan).ok()
                    }
                    "sensor_msgs/msg/PointCloud2" => {
                        let cloud = if topic.contains("ground") {
                            generator.generate_ground_plane()
                        } else {
                            generator.generate_point_cloud(&topic)
                        };
                        rmp_serde::to_vec(&cloud).ok()
                    }
                    "geometry_msgs/msg/PoseStamped" => {
                        let pose = generator.generate_pose(&topic);
                        rmp_serde::to_vec(&pose).ok()
                    }
                    "visualization_msgs/msg/MarkerArray" => {
                        let markers = generator.generate_marker_array(&topic);
                        rmp_serde::to_vec(&markers).ok()
                    }
                    "tf2_msgs/msg/TFMessage" => {
                        let tf = generator.generate_tf_message(&topic);
                        rmp_serde::to_vec(&tf).ok()
                    }
                    "nav_msgs/msg/Odometry" => {
                        let odom = generator.generate_odometry(&topic);
                        rmp_serde::to_vec(&odom).ok()
                    }
                    "sensor_msgs/msg/Imu" => {
                        let imu = generator.generate_imu(&topic);
                        rmp_serde::to_vec(&imu).ok()
                    }
                    "geometry_msgs/msg/Twist" | "geometry_msgs/msg/TwistStamped" => {
                        let twist = generator.generate_twist(&topic);
                        rmp_serde::to_vec(&twist).ok()
                    }
                    "sensor_msgs/msg/JointState" => {
                        let joint_state = generator.generate_joint_state(&topic);
                        rmp_serde::to_vec(&joint_state).ok()
                    }
                    "sensor_msgs/msg/CameraInfo" => {
                        let camera_info = generator.generate_camera_info(&topic);
                        rmp_serde::to_vec(&camera_info).ok()
                    }
                    "rosgraph_msgs/msg/Clock" => {
                        let clock = generator.generate_clock(&topic);
                        rmp_serde::to_vec(&clock).ok()
                    }
                    "std_msgs/msg/String" => {
                        let string_msg = generator.generate_string(&topic);
                        rmp_serde::to_vec(&string_msg).ok()
                    }
                    "rcl_interfaces/msg/Log" => {
                        let log = generator.generate_log(&topic);
                        rmp_serde::to_vec(&log).ok()
                    }
                    _ => None,
                };

                if let Some(payload) = payload {
                    let timestamp = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .map(|d| d.as_secs_f64())
                        .unwrap_or(0.0);

                    mock_state.broadcast(ServerMessage::Data {
                        topic: topic.clone(),
                        msg_type: msg_type.clone(),
                        timestamp,
                        payload,
                    });
                }
            }
        }
    });
}
