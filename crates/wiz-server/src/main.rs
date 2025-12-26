mod bridge;
mod ffi;
mod mock_data;
mod state;
mod ws;

use std::net::SocketAddr;
use std::time::Duration;

use axum::{Router, routing::get};
use tower_http::cors::{Any, CorsLayer};
use tracing::info;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

#[cfg(feature = "ros2")]
use crate::bridge::Ros2Bridge;
use crate::mock_data::MockDataGenerator;
pub use crate::state::AppState;

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
    let mock_state = state.clone();
    tokio::spawn(async move {
        run_mock_data_generator(mock_state).await;
    });
}

#[tokio::main]
async fn main() {
    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "wiz_server=debug,tower_http=debug".into()),
        )
        .with(tracing_subscriber::fmt::layer())
        .init();

    // Initialize state with ROS2 bridge if feature is enabled
    #[cfg(feature = "ros2")]
    let state = {
        info!("Initializing ROS2 bridge...");
        let bridge = Ros2Bridge::new("wiz_server", "");
        if bridge.is_initialized() {
            info!("ROS2 bridge initialized successfully");
            AppState::with_ros2_bridge(bridge)
        } else {
            info!("ROS2 bridge failed to initialize, falling back to mock mode");
            AppState::new()
        }
    };

    #[cfg(not(feature = "ros2"))]
    let state = AppState::new();

    // Spawn appropriate background task based on mode
    if state.has_ros2_bridge() {
        // ROS2 mode: spin the bridge to process callbacks and refresh topics
        #[cfg(feature = "ros2")]
        {
            let ros2_state = state.clone();
            tokio::spawn(async move {
                run_ros2_spin_loop(ros2_state).await;
            });
            info!("ROS2 spin loop started");
        }
    } else {
        // Mock mode: generate fake data for subscribed topics
        spawn_mock_data_generator(state.clone());
        info!("Mock data generator running - subscribe to topics to receive data");
    }

    let app = create_app(state);

    let addr = SocketAddr::from(([0, 0, 0, 0], 9090));
    info!("wiz-server listening on {}", addr);

    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

/// Background task that spins the ROS2 bridge to process callbacks and forwards data
#[cfg(feature = "ros2")]
async fn run_ros2_spin_loop(state: AppState) {
    use wiz_protocol::ServerMessage;

    let bridge = match state.ros2_bridge() {
        Some(b) => b.clone(),
        None => return,
    };

    let mut interval = tokio::time::interval(Duration::from_millis(10)); // 100 Hz

    loop {
        interval.tick().await;

        // Spin ROS2 node to process callbacks
        state.spin_once();

        // Check for new data on subscribed topics and forward to clients
        let subscriptions = state.get_subscriptions();

        for (topic, msg_type) in subscriptions {
            let payload = match msg_type.as_str() {
                "sensor_msgs/msg/LaserScan" => {
                    // Get laser scan data from bridge (returns None if no new data)
                    bridge
                        .get_laserscan(&topic)
                        .and_then(|scan| rmp_serde::to_vec(&scan).ok())
                }
                "sensor_msgs/msg/PointCloud2" => {
                    // Get point cloud data from bridge (returns None if no new data)
                    bridge
                        .get_pointcloud2(&topic)
                        .and_then(|cloud| rmp_serde::to_vec(&cloud).ok())
                }
                // Other message types can be added here as needed
                _ => None,
            };

            if let Some(payload) = payload {
                let timestamp = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_secs_f64())
                    .unwrap_or(0.0);

                state.broadcast(ServerMessage::Data {
                    topic: topic.clone(),
                    msg_type: msg_type.clone(),
                    timestamp,
                    payload,
                });
            }
        }
    }
}

/// Background task that generates and broadcasts mock data for subscribed topics
async fn run_mock_data_generator(state: AppState) {
    use wiz_protocol::ServerMessage;

    let mut generator = MockDataGenerator::new();
    let mut interval = tokio::time::interval(Duration::from_millis(100)); // 10 Hz

    loop {
        interval.tick().await;
        generator.tick();

        // Get active subscriptions and send mock data
        let subscriptions = state.get_subscriptions();

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

                state.broadcast(ServerMessage::Data {
                    topic: topic.clone(),
                    msg_type: msg_type.clone(),
                    timestamp,
                    payload,
                });
            }
        }
    }
}
