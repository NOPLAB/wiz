use futures_util::{SinkExt, StreamExt};
use parking_lot::Mutex;
use std::sync::Arc;
use tokio::sync::mpsc;
use tokio_tungstenite::{connect_async, tungstenite::Message};
use wiz_protocol::{decode_server_message, encode_client_message, ClientMessage, ServerMessage};

pub struct WsClient {
    tx: mpsc::Sender<ClientMessage>,
    rx: Arc<Mutex<mpsc::Receiver<ServerMessage>>>,
}

impl WsClient {
    pub async fn connect(url: &str) -> Result<Self, String> {
        let (ws_stream, _) = connect_async(url)
            .await
            .map_err(|e| format!("Failed to connect: {}", e))?;

        let (mut write, mut read) = ws_stream.split();

        let (client_tx, mut client_rx) = mpsc::channel::<ClientMessage>(32);
        let (server_tx, server_rx) = mpsc::channel::<ServerMessage>(32);

        // Task to send messages to server
        tokio::spawn(async move {
            while let Some(msg) = client_rx.recv().await {
                if let Ok(data) = encode_client_message(&msg) {
                    if write.send(Message::Binary(data.into())).await.is_err() {
                        break;
                    }
                }
            }
        });

        // Task to receive messages from server
        tokio::spawn(async move {
            while let Some(Ok(msg)) = read.next().await {
                if let Message::Binary(data) = msg {
                    if let Ok(server_msg) = decode_server_message(&data) {
                        if server_tx.send(server_msg).await.is_err() {
                            break;
                        }
                    }
                }
            }
        });

        Ok(Self {
            tx: client_tx,
            rx: Arc::new(Mutex::new(server_rx)),
        })
    }

    pub async fn send(&self, msg: ClientMessage) -> Result<(), String> {
        self.tx
            .send(msg)
            .await
            .map_err(|e| format!("Failed to send: {}", e))
    }

    pub fn try_recv(&self) -> Option<ServerMessage> {
        self.rx.lock().try_recv().ok()
    }

    pub async fn subscribe(
        &self,
        id: String,
        topic: String,
        msg_type: String,
        throttle_rate: Option<u32>,
    ) -> Result<(), String> {
        self.send(ClientMessage::Subscribe {
            id,
            topic,
            msg_type,
            throttle_rate,
        })
        .await
    }

    pub async fn unsubscribe(&self, id: String) -> Result<(), String> {
        self.send(ClientMessage::Unsubscribe { id }).await
    }

    pub async fn list_topics(&self) -> Result<(), String> {
        self.send(ClientMessage::ListTopics).await
    }
}
