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

use crate::mock_data::MockDataGenerator;
use crate::state::AppState;

#[tokio::main]
async fn main() {
    tracing_subscriber::registry()
        .with(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "wiz_server=debug,tower_http=debug".into()),
        )
        .with(tracing_subscriber::fmt::layer())
        .init();

    let state = AppState::new();

    // Spawn mock data generator task
    let mock_state = state.clone();
    tokio::spawn(async move {
        run_mock_data_generator(mock_state).await;
    });

    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    let app = Router::new()
        .route("/ws", get(ws::ws_handler))
        .route("/health", get(|| async { "OK" }))
        .layer(cors)
        .with_state(state);

    let addr = SocketAddr::from(([0, 0, 0, 0], 9090));
    info!("wiz-server listening on {}", addr);
    info!("Mock data generator running - subscribe to topics to receive data");

    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
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
