use futures_util::{SinkExt, StreamExt};
use parking_lot::Mutex;
use std::sync::Arc;
use tokio::sync::mpsc;
use tokio_tungstenite::{connect_async, tungstenite::Message};
use tracing::{debug, error, info};
use wiz_protocol::{ClientMessage, ServerMessage, decode_server_message, encode_client_message};

/// Connection command sent from UI to background task
#[derive(Debug)]
pub enum WsCommand {
    Connect(String),
    Disconnect,
    Send(ClientMessage),
}

/// Connection state
#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Error(String),
}

/// Event received from background task
#[derive(Debug)]
pub enum WsEvent {
    Connected,
    Disconnected,
    Message(ServerMessage),
    Error(String),
}

/// WebSocket client handle for use from GUI
pub struct WsClientHandle {
    command_tx: mpsc::UnboundedSender<WsCommand>,
    event_rx: Arc<Mutex<mpsc::UnboundedReceiver<WsEvent>>>,
    state: Arc<Mutex<ConnectionState>>,
}

impl WsClientHandle {
    /// Create a new WebSocket client handle
    /// This spawns a background task that manages the connection
    pub fn new() -> Self {
        let (command_tx, command_rx) = mpsc::unbounded_channel();
        let (event_tx, event_rx) = mpsc::unbounded_channel();
        let state = Arc::new(Mutex::new(ConnectionState::Disconnected));

        // Spawn background task
        let state_clone = state.clone();
        std::thread::spawn(move || {
            let rt = tokio::runtime::Runtime::new().expect("Failed to create tokio runtime");
            rt.block_on(ws_background_task(command_rx, event_tx, state_clone));
        });

        Self {
            command_tx,
            event_rx: Arc::new(Mutex::new(event_rx)),
            state,
        }
    }

    /// Connect to the WebSocket server
    pub fn connect(&self, url: &str) {
        let _ = self.command_tx.send(WsCommand::Connect(url.to_string()));
    }

    /// Disconnect from the WebSocket server
    pub fn disconnect(&self) {
        let _ = self.command_tx.send(WsCommand::Disconnect);
    }

    /// Send a client message
    pub fn send(&self, msg: ClientMessage) {
        let _ = self.command_tx.send(WsCommand::Send(msg));
    }

    /// Subscribe to a topic
    #[allow(dead_code)]
    pub fn subscribe(
        &self,
        id: String,
        topic: String,
        msg_type: String,
        throttle_rate: Option<u32>,
    ) {
        self.send(ClientMessage::Subscribe {
            id,
            topic,
            msg_type,
            throttle_rate,
        });
    }

    /// Unsubscribe from a topic
    #[allow(dead_code)]
    pub fn unsubscribe(&self, id: String) {
        self.send(ClientMessage::Unsubscribe { id });
    }

    /// Request topic list
    pub fn list_topics(&self) {
        self.send(ClientMessage::ListTopics);
    }

    /// Poll for events (non-blocking)
    pub fn poll_events(&self) -> Vec<WsEvent> {
        let mut events = Vec::new();
        let mut rx = self.event_rx.lock();
        while let Ok(event) = rx.try_recv() {
            events.push(event);
        }
        events
    }

    /// Get current connection state
    pub fn state(&self) -> ConnectionState {
        self.state.lock().clone()
    }

    /// Check if connected
    #[allow(dead_code)]
    pub fn is_connected(&self) -> bool {
        matches!(*self.state.lock(), ConnectionState::Connected)
    }
}

impl Default for WsClientHandle {
    fn default() -> Self {
        Self::new()
    }
}

/// Background task that manages the WebSocket connection
async fn ws_background_task(
    mut command_rx: mpsc::UnboundedReceiver<WsCommand>,
    event_tx: mpsc::UnboundedSender<WsEvent>,
    state: Arc<Mutex<ConnectionState>>,
) {
    let mut ws_tx: Option<mpsc::UnboundedSender<ClientMessage>> = None;

    loop {
        tokio::select! {
            Some(cmd) = command_rx.recv() => {
                match cmd {
                    WsCommand::Connect(url) => {
                        info!("Connecting to {}", url);
                        *state.lock() = ConnectionState::Connecting;

                        match connect_to_server(&url, event_tx.clone()).await {
                            Ok(tx) => {
                                ws_tx = Some(tx);
                                *state.lock() = ConnectionState::Connected;
                                let _ = event_tx.send(WsEvent::Connected);
                                info!("Connected to {}", url);
                            }
                            Err(e) => {
                                let error_msg = e.to_string();
                                *state.lock() = ConnectionState::Error(error_msg.clone());
                                let _ = event_tx.send(WsEvent::Error(error_msg));
                                error!("Failed to connect: {}", e);
                            }
                        }
                    }
                    WsCommand::Disconnect => {
                        info!("Disconnecting");
                        ws_tx = None;
                        *state.lock() = ConnectionState::Disconnected;
                        let _ = event_tx.send(WsEvent::Disconnected);
                    }
                    WsCommand::Send(msg) => {
                        if let Some(ref tx) = ws_tx {
                            debug!("Sending message: {:?}", msg);
                            let _ = tx.send(msg);
                        }
                    }
                }
            }
            else => break,
        }
    }
}

async fn connect_to_server(
    url: &str,
    event_tx: mpsc::UnboundedSender<WsEvent>,
) -> Result<mpsc::UnboundedSender<ClientMessage>, String> {
    let (ws_stream, _) = connect_async(url)
        .await
        .map_err(|e| format!("Failed to connect: {}", e))?;

    let (mut write, mut read) = ws_stream.split();

    let (client_tx, mut client_rx) = mpsc::unbounded_channel::<ClientMessage>();

    // Task to send messages to server
    tokio::spawn(async move {
        while let Some(msg) = client_rx.recv().await {
            if let Ok(data) = encode_client_message(&msg)
                && write.send(Message::Binary(data.into())).await.is_err()
            {
                break;
            }
        }
    });

    // Task to receive messages from server
    tokio::spawn(async move {
        while let Some(msg_result) = read.next().await {
            match msg_result {
                Ok(Message::Binary(data)) => {
                    if let Ok(server_msg) = decode_server_message(&data)
                        && event_tx.send(WsEvent::Message(server_msg)).is_err()
                    {
                        break;
                    }
                }
                Ok(Message::Close(_)) => {
                    let _ = event_tx.send(WsEvent::Disconnected);
                    break;
                }
                Err(e) => {
                    let _ = event_tx.send(WsEvent::Error(e.to_string()));
                    break;
                }
                _ => {}
            }
        }
    });

    Ok(client_tx)
}
