use ewebsock::{WsEvent as EwebsockEvent, WsMessage, WsReceiver, WsSender};
use parking_lot::Mutex;
use std::sync::Arc;
use tracing::{debug, error, info, warn};
use wiz_protocol::{ClientMessage, ServerMessage, decode_server_message, encode_client_message};

/// Connection state
#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Error(String),
}

/// Event received from WebSocket
#[derive(Debug)]
pub enum WsEvent {
    Connected,
    Disconnected,
    Message(ServerMessage),
    Error(String),
}

/// Shared connection state
struct ConnectionInner {
    state: ConnectionState,
    sender: Option<WsSender>,
    receiver: Option<WsReceiver>,
    pending_events: Vec<WsEvent>,
}

/// WebSocket client handle for use from GUI
/// Uses ewebsock which works on both native and WASM
pub struct WsClientHandle {
    inner: Arc<Mutex<ConnectionInner>>,
}

impl WsClientHandle {
    /// Create a new WebSocket client handle
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(ConnectionInner {
                state: ConnectionState::Disconnected,
                sender: None,
                receiver: None,
                pending_events: Vec::new(),
            })),
        }
    }

    /// Connect to the WebSocket server
    pub fn connect(&self, url: &str) {
        info!("Connecting to {}", url);

        let mut inner = self.inner.lock();
        inner.state = ConnectionState::Connecting;

        // Close existing connection if any
        if let Some(mut sender) = inner.sender.take() {
            sender.close();
        }
        inner.receiver = None;

        match ewebsock::connect(url, ewebsock::Options::default()) {
            Ok((sender, receiver)) => {
                inner.sender = Some(sender);
                inner.receiver = Some(receiver);
                // Note: actual Connected state will be set when we receive WsEvent::Opened
            }
            Err(e) => {
                let error_msg = e.to_string();
                error!("Failed to connect: {}", error_msg);
                inner.state = ConnectionState::Error(error_msg.clone());
                inner.pending_events.push(WsEvent::Error(error_msg));
            }
        }
    }

    /// Disconnect from the WebSocket server
    pub fn disconnect(&self) {
        info!("Disconnecting");
        let mut inner = self.inner.lock();

        if let Some(mut sender) = inner.sender.take() {
            sender.close();
        }
        inner.receiver = None;
        inner.state = ConnectionState::Disconnected;
        inner.pending_events.push(WsEvent::Disconnected);
    }

    /// Send a client message
    pub fn send(&self, msg: ClientMessage) {
        let mut inner = self.inner.lock();

        if let Some(ref mut sender) = inner.sender {
            match encode_client_message(&msg) {
                Ok(data) => {
                    debug!("Sending message: {:?}", msg);
                    sender.send(WsMessage::Binary(data));
                }
                Err(e) => {
                    error!("Failed to encode message: {}", e);
                }
            }
        } else {
            warn!("Cannot send message: not connected");
        }
    }

    /// Subscribe to a topic
    pub fn subscribe(&self, topic: &str, msg_type: &str, throttle_rate: Option<u32>) {
        // Use topic name as subscription ID for simplicity
        let id = topic.to_string();
        self.send(ClientMessage::Subscribe {
            id,
            topic: topic.to_string(),
            msg_type: msg_type.to_string(),
            throttle_rate,
        });
    }

    /// Unsubscribe from a subscription by ID
    pub fn unsubscribe(&self, subscription_id: u32) {
        // We don't have the original topic name, so we'll need to track this separately
        // For now, just send the unsubscribe with the subscription ID as string
        self.send(ClientMessage::Unsubscribe {
            id: subscription_id.to_string(),
        });
    }

    /// Unsubscribe by topic name
    #[allow(dead_code)]
    pub fn unsubscribe_topic(&self, topic: &str) {
        self.send(ClientMessage::Unsubscribe {
            id: topic.to_string(),
        });
    }

    /// Request topic list
    pub fn list_topics(&self) {
        self.send(ClientMessage::ListTopics);
    }

    /// Poll for events (non-blocking)
    /// This should be called every frame to process incoming messages
    pub fn poll_events(&self) -> Vec<WsEvent> {
        let mut inner = self.inner.lock();
        let mut events = std::mem::take(&mut inner.pending_events);

        // Take the receiver temporarily to avoid borrow conflicts
        if let Some(receiver) = inner.receiver.take() {
            // Collect all incoming events
            let mut received_events = Vec::new();
            while let Some(event) = receiver.try_recv() {
                received_events.push(event);
            }

            // Put the receiver back
            inner.receiver = Some(receiver);

            // Process the collected events
            for event in received_events {
                match event {
                    EwebsockEvent::Opened => {
                        info!("WebSocket connected");
                        inner.state = ConnectionState::Connected;
                        events.push(WsEvent::Connected);
                    }
                    EwebsockEvent::Message(WsMessage::Binary(data)) => {
                        match decode_server_message(&data) {
                            Ok(server_msg) => {
                                events.push(WsEvent::Message(server_msg));
                            }
                            Err(e) => {
                                warn!("Failed to decode server message: {}", e);
                            }
                        }
                    }
                    EwebsockEvent::Message(WsMessage::Text(text)) => {
                        debug!("Received text message (ignoring): {}", text);
                    }
                    EwebsockEvent::Message(WsMessage::Ping(_) | WsMessage::Pong(_)) => {
                        // Ping/pong handled by library
                    }
                    EwebsockEvent::Message(WsMessage::Unknown(_)) => {
                        warn!("Received unknown message type");
                    }
                    EwebsockEvent::Error(e) => {
                        error!("WebSocket error: {}", e);
                        inner.state = ConnectionState::Error(e.clone());
                        events.push(WsEvent::Error(e));
                    }
                    EwebsockEvent::Closed => {
                        info!("WebSocket closed");
                        inner.state = ConnectionState::Disconnected;
                        inner.sender = None;
                        events.push(WsEvent::Disconnected);
                    }
                }
            }
        }

        events
    }

    /// Get current connection state
    pub fn state(&self) -> ConnectionState {
        self.inner.lock().state.clone()
    }

    /// Check if connected
    #[allow(dead_code)]
    pub fn is_connected(&self) -> bool {
        matches!(self.inner.lock().state, ConnectionState::Connected)
    }
}

impl Default for WsClientHandle {
    fn default() -> Self {
        Self::new()
    }
}
