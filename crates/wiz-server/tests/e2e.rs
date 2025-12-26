//! End-to-end tests for wiz-server
//!
//! These tests verify the server's WebSocket protocol and HTTP endpoints
//! by starting a real server instance and connecting to it.

use std::net::SocketAddr;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use tokio::net::TcpListener;
use tokio_tungstenite::{connect_async, tungstenite::Message};
use wiz_protocol::{ClientMessage, ServerMessage, decode_server_message, encode_client_message};
use wiz_server::{AppState, create_app, spawn_mock_data_generator};

/// Test helper: Start a server on a random available port and return the address
async fn start_test_server() -> SocketAddr {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();

    let state = AppState::new();
    spawn_mock_data_generator(state.clone());

    let app = create_app(state);

    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });

    // Give the server a moment to start
    tokio::time::sleep(Duration::from_millis(50)).await;

    addr
}

/// Test helper: Connect to WebSocket and return the stream
async fn connect_ws(
    addr: SocketAddr,
) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>> {
    let url = format!("ws://{addr}/ws");
    let (ws_stream, _) = connect_async(&url).await.expect("Failed to connect");
    ws_stream
}

/// Test helper: Send a client message and receive the next server message
async fn send_and_receive(
    ws: &mut tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    msg: ClientMessage,
) -> ServerMessage {
    let data = encode_client_message(&msg).expect("Failed to encode");
    ws.send(Message::Binary(data.into()))
        .await
        .expect("Failed to send");

    // Wait for response with timeout
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            if let Some(Ok(Message::Binary(data))) = ws.next().await {
                if let Ok(msg) = decode_server_message(&data) {
                    return msg;
                }
            }
        }
    })
    .await
    .expect("Timeout waiting for response")
}

// =============================================================================
// Health Check Tests
// =============================================================================

#[tokio::test]
async fn test_health_endpoint() {
    let addr = start_test_server().await;
    let url = format!("http://{addr}/health");

    let response = reqwest::get(&url).await.expect("Failed to request health");
    assert!(response.status().is_success());

    let body = response.text().await.expect("Failed to read body");
    assert_eq!(body, "OK");
}

// =============================================================================
// WebSocket Connection Tests
// =============================================================================

#[tokio::test]
async fn test_websocket_connection() {
    let addr = start_test_server().await;
    let ws = connect_ws(addr).await;

    // Connection successful, just verify we can close it cleanly
    drop(ws);
}

#[tokio::test]
async fn test_websocket_multiple_connections() {
    let addr = start_test_server().await;

    // Connect multiple clients simultaneously
    let ws1 = connect_ws(addr).await;
    let ws2 = connect_ws(addr).await;
    let ws3 = connect_ws(addr).await;

    // All connections should be active
    drop(ws1);
    drop(ws2);
    drop(ws3);
}

// =============================================================================
// ListTopics Tests
// =============================================================================

#[tokio::test]
async fn test_list_topics() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    let response = send_and_receive(&mut ws, ClientMessage::ListTopics).await;

    match response {
        ServerMessage::Topics { topics } => {
            assert!(!topics.is_empty(), "Topics list should not be empty");

            // Verify expected mock topics exist
            let topic_names: Vec<_> = topics.iter().map(|t| t.name.as_str()).collect();
            assert!(topic_names.contains(&"/scan"), "Should contain /scan topic");
            assert!(
                topic_names.contains(&"/velodyne_points"),
                "Should contain /velodyne_points topic"
            );
        }
        other => panic!("Expected Topics response, got: {other:?}"),
    }
}

// =============================================================================
// Subscribe/Unsubscribe Tests
// =============================================================================

#[tokio::test]
async fn test_subscribe_to_topic() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    let sub_id = "test-sub-1".to_string();
    let topic = "/scan".to_string();
    let msg_type = "sensor_msgs/msg/LaserScan".to_string();

    let response = send_and_receive(
        &mut ws,
        ClientMessage::Subscribe {
            id: sub_id.clone(),
            topic: topic.clone(),
            msg_type,
            throttle_rate: None,
        },
    )
    .await;

    match response {
        ServerMessage::Subscribed { id, topic: t } => {
            assert_eq!(id, sub_id);
            assert_eq!(t, topic);
        }
        other => panic!("Expected Subscribed response, got: {other:?}"),
    }
}

#[tokio::test]
async fn test_unsubscribe_from_topic() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    let sub_id = "test-sub-2".to_string();

    // First subscribe
    let _ = send_and_receive(
        &mut ws,
        ClientMessage::Subscribe {
            id: sub_id.clone(),
            topic: "/scan".to_string(),
            msg_type: "sensor_msgs/msg/LaserScan".to_string(),
            throttle_rate: None,
        },
    )
    .await;

    // Then unsubscribe
    let response =
        send_and_receive(&mut ws, ClientMessage::Unsubscribe { id: sub_id.clone() }).await;

    match response {
        ServerMessage::Unsubscribed { id } => {
            assert_eq!(id, sub_id);
        }
        other => panic!("Expected Unsubscribed response, got: {other:?}"),
    }
}

#[tokio::test]
async fn test_subscribe_multiple_topics() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    // Subscribe to multiple topics
    let subscriptions = vec![
        ("sub-1", "/scan", "sensor_msgs/msg/LaserScan"),
        ("sub-2", "/velodyne_points", "sensor_msgs/msg/PointCloud2"),
        ("sub-3", "/robot_pose", "geometry_msgs/msg/PoseStamped"),
    ];

    for (id, topic, msg_type) in subscriptions {
        let response = send_and_receive(
            &mut ws,
            ClientMessage::Subscribe {
                id: id.to_string(),
                topic: topic.to_string(),
                msg_type: msg_type.to_string(),
                throttle_rate: None,
            },
        )
        .await;

        match response {
            ServerMessage::Subscribed { id: rid, topic: t } => {
                assert_eq!(rid, id);
                assert_eq!(t, topic);
            }
            other => panic!("Expected Subscribed response, got: {other:?}"),
        }
    }
}

// =============================================================================
// TF Lookup Tests
// =============================================================================

#[tokio::test]
async fn test_tf_lookup() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    let response = send_and_receive(
        &mut ws,
        ClientMessage::TfLookup {
            target_frame: "base_link".to_string(),
            source_frame: "map".to_string(),
        },
    )
    .await;

    match response {
        ServerMessage::Transform {
            target_frame,
            source_frame,
            transform,
        } => {
            assert_eq!(target_frame, "base_link");
            assert_eq!(source_frame, "map");
            // Transform should be identity in mock mode
            assert_eq!(transform.translation[0], 0.0);
            assert_eq!(transform.translation[1], 0.0);
            assert_eq!(transform.translation[2], 0.0);
        }
        other => panic!("Expected Transform response, got: {other:?}"),
    }
}

// =============================================================================
// Data Streaming Tests
// =============================================================================

#[tokio::test]
async fn test_receive_laser_scan_data() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    // Subscribe to laser scan
    let _ = send_and_receive(
        &mut ws,
        ClientMessage::Subscribe {
            id: "scan-sub".to_string(),
            topic: "/scan".to_string(),
            msg_type: "sensor_msgs/msg/LaserScan".to_string(),
            throttle_rate: None,
        },
    )
    .await;

    // Wait for data message (mock generator runs at 10Hz)
    let data_received = tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            if let Some(Ok(Message::Binary(data))) = ws.next().await {
                if let Ok(ServerMessage::Data {
                    topic,
                    msg_type,
                    payload,
                    ..
                }) = decode_server_message(&data)
                {
                    if topic == "/scan" && msg_type == "sensor_msgs/msg/LaserScan" {
                        return payload;
                    }
                }
            }
        }
    })
    .await;

    assert!(data_received.is_ok(), "Should receive LaserScan data");
    let payload = data_received.unwrap();
    assert!(!payload.is_empty(), "Payload should not be empty");
}

#[tokio::test]
async fn test_receive_point_cloud_data() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    // Subscribe to point cloud
    let _ = send_and_receive(
        &mut ws,
        ClientMessage::Subscribe {
            id: "cloud-sub".to_string(),
            topic: "/velodyne_points".to_string(),
            msg_type: "sensor_msgs/msg/PointCloud2".to_string(),
            throttle_rate: None,
        },
    )
    .await;

    // Wait for data message
    let data_received = tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            if let Some(Ok(Message::Binary(data))) = ws.next().await {
                if let Ok(ServerMessage::Data {
                    topic,
                    msg_type,
                    payload,
                    ..
                }) = decode_server_message(&data)
                {
                    if topic == "/velodyne_points" && msg_type == "sensor_msgs/msg/PointCloud2" {
                        return payload;
                    }
                }
            }
        }
    })
    .await;

    assert!(data_received.is_ok(), "Should receive PointCloud2 data");
    let payload = data_received.unwrap();
    assert!(!payload.is_empty(), "Payload should not be empty");
}

#[tokio::test]
async fn test_receive_pose_data() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    // Subscribe to robot pose
    let _ = send_and_receive(
        &mut ws,
        ClientMessage::Subscribe {
            id: "pose-sub".to_string(),
            topic: "/robot_pose".to_string(),
            msg_type: "geometry_msgs/msg/PoseStamped".to_string(),
            throttle_rate: None,
        },
    )
    .await;

    // Wait for data message
    let data_received = tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            if let Some(Ok(Message::Binary(data))) = ws.next().await {
                if let Ok(ServerMessage::Data {
                    topic,
                    msg_type,
                    payload,
                    ..
                }) = decode_server_message(&data)
                {
                    if topic == "/robot_pose" && msg_type == "geometry_msgs/msg/PoseStamped" {
                        return payload;
                    }
                }
            }
        }
    })
    .await;

    assert!(data_received.is_ok(), "Should receive PoseStamped data");
    let payload = data_received.unwrap();
    assert!(!payload.is_empty(), "Payload should not be empty");
}

// =============================================================================
// Protocol Tests
// =============================================================================

#[tokio::test]
async fn test_invalid_message_handling() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    // Send invalid binary data
    ws.send(Message::Binary(vec![0, 1, 2, 3, 4].into()))
        .await
        .expect("Failed to send");

    // Server should not crash, connection should still work
    // Send a valid message to verify
    let response = send_and_receive(&mut ws, ClientMessage::ListTopics).await;

    match response {
        ServerMessage::Topics { .. } => {}
        other => panic!("Expected Topics response after invalid message, got: {other:?}"),
    }
}

#[tokio::test]
async fn test_text_message_handling() {
    let addr = start_test_server().await;
    let mut ws = connect_ws(addr).await;

    // Send text message (server accepts but prefers binary)
    ws.send(Message::Text("test".to_string().into()))
        .await
        .expect("Failed to send");

    // Server should not crash, send valid message to verify
    let response = send_and_receive(&mut ws, ClientMessage::ListTopics).await;

    match response {
        ServerMessage::Topics { .. } => {}
        other => panic!("Expected Topics response after text message, got: {other:?}"),
    }
}
