use crate::shared_code::controller::ControllerState;

macro_rules! discriminant_conversion {
    ($t:ty { $( $discrim:expr => $enum:pat => $def:expr ),* }) => {
        impl<'a> From<&$t> for u32 {
            fn from(value: &$t) -> u32 {
                match value {
                    $( $enum => $discrim, )*
                }
            }
        }

        impl<'a> TryFrom<u32> for $t {
            type Error = ();

            fn try_from(value: u32) -> Result<$t, Self::Error> {
                match value {
                    $( $discrim => Ok($def), )*
                    _ => Err(()),
                }
            }
        }
    };
}

pub enum Message<'a> {
    ControllerData(u32, ControllerState, ControllerState),
    LogData(u32, &'a [u8]),
    MissedLogData(u32),
    ForgotLogData(u32),
}

discriminant_conversion!(Message<'a> {
    0 => Message::ControllerData(..) => Message::ControllerData(0, ControllerState::default(), ControllerState::default()),
    1 => Message::LogData(..) => Message::LogData(0, &[]),
    2 => Message::MissedLogData(..) => Message::MissedLogData(0),
    3 => Message::ForgotLogData(..) => Message::ForgotLogData(0)
});

impl<'a> Message<'a> {
    pub fn buffer_len(&self) -> usize {
        match self {
            Self::ControllerData(..) => 36,
            Self::LogData(_, data) => 12 + data.len(),
            Self::MissedLogData(..) => 12,
            Self::ForgotLogData(..) => 12,
        }
    }

    pub fn to_le_bytes(&self, buffer: &mut [u8]) -> u32 {
        let discriminant: u32 = self.into();
        buffer[0..4].copy_from_slice(&discriminant.to_le_bytes());
        let len: u32 = match self {
            Self::ControllerData(id, primary_state, secondary_state) => {
                buffer[8..12].copy_from_slice(&id.to_le_bytes());
                buffer[12..24].copy_from_slice(&primary_state.to_le_bytes());
                buffer[24..36].copy_from_slice(&secondary_state.to_le_bytes());
                36
            }
            Self::LogData(id, data) => {
                buffer[8..12].copy_from_slice(&id.to_le_bytes());
                buffer[12..(12 + data.len())].copy_from_slice(data);
                12 + data.len() as u32
            }
            Self::MissedLogData(id) => {
                buffer[8..12].copy_from_slice(&id.to_le_bytes());
                12
            }
            Self::ForgotLogData(id) => {
                buffer[8..12].copy_from_slice(&id.to_le_bytes());
                12
            }
        };
        buffer[4..8].copy_from_slice(&len.to_le_bytes());
        len
    }

    pub fn from_le_bytes(buffer: &'a [u8]) -> (usize, Option<Self>) {
        if buffer.len() < 12 {
            return (buffer.len(), None);
        }
        let id = u32::from_le_bytes([buffer[8], buffer[9], buffer[10], buffer[11]]);
        let len = u32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]) as usize;
        (
            len,
            match u32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]])
                .try_into()
                .ok()
            {
                Some(Self::ControllerData(..)) => {
                    if buffer.len() > 36 {
                        Some(Self::ControllerData(
                            id,
                            ControllerState::from_le_bytes(buffer[12..24].try_into().unwrap()),
                            ControllerState::from_le_bytes(buffer[24..36].try_into().unwrap()),
                        ))
                    } else {
                        return (buffer.len(), None);
                    }
                }
                Some(Self::LogData(..)) => {
                    if buffer.len() >= len {
                        Some(Self::LogData(id, &buffer[12..len]))
                    } else {
                        return (buffer.len(), Some(Self::LogData(id, &buffer[12..])));
                    }
                }
                Some(Self::MissedLogData(..)) => Some(Self::MissedLogData(id)),
                Some(Self::ForgotLogData(..)) => Some(Self::ForgotLogData(id)),
                None => None,
            },
        )
    }
}
