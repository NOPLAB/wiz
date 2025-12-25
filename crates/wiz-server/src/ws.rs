use axum::{
    extract::{
        ws::{Message, WebSocket},
        State, WebSocketUpgrade,
    },
    response::IntoResponse,
};
use futures_util::{SinkExt, StreamExt};
use tracing::{debug, error, info};
use wiz_protocol::{decode_client_message, encode_server_message, ClientMessage, ServerMessage};

use crate::state::AppState;

pub async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
) -> impl IntoResponse {
    ws.on_upgrade(|socket| handle_socket(socket, state))
}

async fn handle_socket(socket: WebSocket, state: AppState) {
    let (mut sender, mut receiver) = socket.split();

    let mut rx = state.subscribe();

    // Task to forward broadcast messages to the client
    let send_task = tokio::spawn(async move {
        while let Ok(msg) = rx.recv().await {
            if let Ok(data) = encode_server_message(&msg) {
                if sender.send(Message::Binary(data.into())).await.is_err() {
                    break;
                }
            }
        }
    });

    // Handle incoming messages from client
    let state_clone = state.clone();
    let recv_task = tokio::spawn(async move {
        while let Some(Ok(msg)) = receiver.next().await {
            match msg {
                Message::Binary(data) => {
                    if let Err(e) = handle_client_message(&state_clone, &data).await {
                        error!("Error handling client message: {}", e);
                    }
                }
                Message::Text(text) => {
                    // Try to parse as JSON for debugging, but prefer binary
                    debug!("Received text message: {}", text);
                }
                Message::Close(_) => {
                    info!("Client disconnected");
                    break;
                }
                _ => {}
            }
        }
    });

    // Wait for either task to complete
    tokio::select! {
        _ = send_task => {},
        _ = recv_task => {},
    }
}

async fn handle_client_message(state: &AppState, data: &[u8]) -> Result<(), String> {
    let msg = decode_client_message(data).map_err(|e| e.to_string())?;

    match msg {
        ClientMessage::Subscribe {
            id,
            topic,
            msg_type,
            throttle_rate,
        } => {
            info!("Subscribe request: {} -> {}", id, topic);
            state.add_subscription(id.clone(), topic.clone(), msg_type, throttle_rate);
            state.broadcast(ServerMessage::Subscribed { id, topic });
        }
        ClientMessage::Unsubscribe { id } => {
            info!("Unsubscribe request: {}", id);
            state.remove_subscription(&id);
            state.broadcast(ServerMessage::Unsubscribed { id });
        }
        ClientMessage::TfLookup {
            target_frame,
            source_frame,
        } => {
            info!("TF lookup: {} -> {}", source_frame, target_frame);
            // TODO: Implement actual TF lookup via ROS2 bridge
            state.broadcast(ServerMessage::Transform {
                target_frame,
                source_frame,
                transform: wiz_core::Transform3D::identity(),
            });
        }
        ClientMessage::ListTopics => {
            info!("List topics request");
            let topics = state.list_topics();
            state.broadcast(ServerMessage::Topics { topics });
        }
    }

    Ok(())
}
