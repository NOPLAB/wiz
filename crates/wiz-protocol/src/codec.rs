use crate::{ClientMessage, ServerMessage};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum CodecError {
    #[error("Failed to encode message: {0}")]
    EncodeError(#[from] rmp_serde::encode::Error),
    #[error("Failed to decode message: {0}")]
    DecodeError(#[from] rmp_serde::decode::Error),
}

pub fn encode_client_message(msg: &ClientMessage) -> Result<Vec<u8>, CodecError> {
    rmp_serde::to_vec(msg).map_err(CodecError::from)
}

pub fn decode_client_message(data: &[u8]) -> Result<ClientMessage, CodecError> {
    rmp_serde::from_slice(data).map_err(CodecError::from)
}

pub fn encode_server_message(msg: &ServerMessage) -> Result<Vec<u8>, CodecError> {
    rmp_serde::to_vec(msg).map_err(CodecError::from)
}

pub fn decode_server_message(data: &[u8]) -> Result<ServerMessage, CodecError> {
    rmp_serde::from_slice(data).map_err(CodecError::from)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_message_roundtrip() {
        let msg = ClientMessage::Subscribe {
            id: "test-id".to_string(),
            topic: "/scan".to_string(),
            msg_type: "sensor_msgs/LaserScan".to_string(),
            throttle_rate: Some(30),
        };

        let encoded = encode_client_message(&msg).unwrap();
        let decoded = decode_client_message(&encoded).unwrap();

        match decoded {
            ClientMessage::Subscribe {
                id,
                topic,
                msg_type,
                throttle_rate,
            } => {
                assert_eq!(id, "test-id");
                assert_eq!(topic, "/scan");
                assert_eq!(msg_type, "sensor_msgs/LaserScan");
                assert_eq!(throttle_rate, Some(30));
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_server_message_roundtrip() {
        let msg = ServerMessage::Error {
            message: "Test error".to_string(),
        };

        let encoded = encode_server_message(&msg).unwrap();
        let decoded = decode_server_message(&encoded).unwrap();

        match decoded {
            ServerMessage::Error { message } => {
                assert_eq!(message, "Test error");
            }
            _ => panic!("Wrong message type"),
        }
    }
}
