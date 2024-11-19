#![allow(dead_code)]

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct StickState {
    x: i16, // divided by 0x8000 (32768)
    y: i16,
}

impl StickState {
    pub fn new(x: f32, y: f32) -> Self {
        Self {
            x: (x * 32768_f32) as i16,
            y: (y * 32768_f32) as i16,
        }
    }

    pub fn from_bytes(bytes: &[u8; 4]) -> Self {
        Self {
            x: i16::from_le_bytes([bytes[0], bytes[1]]),
            y: i16::from_le_bytes([bytes[2], bytes[3]]),
        }
    }

    pub fn to_bytes(&self) -> [u8; 4] {
        let x = self.x.to_le_bytes();
        let y = self.y.to_le_bytes();
        [x[0], x[1], y[0], y[1]]
    }

    pub fn get_x(&self) -> f32 {
        self.x as f32 / 32768_f32
    }

    pub fn get_y(&self) -> f32 {
        self.y as f32 / 32768_f32
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Button {
    Cross,
    Circle,
    Square,
    Triangle,

    Up,
    Down,
    Left,
    Right,

    LeftBumper,
    RightBumper,

    L3,
    R3,

    Select,
    Start,

    Logo,
}

impl Button {
    fn mask(&self) -> u16 {
        1 << *self as usize
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct ControllerState {
    pub left_stick: StickState,
    pub right_stick: StickState,
    pub left_trigger: u8, // divided by 0x100 (256)
    pub right_trigger: u8,
    buttons: u16,
}

impl ControllerState {
    pub fn get(&self, button: Button) -> bool {
        self.buttons & button.mask() != 0
    }

    pub fn set(&mut self, button: Button) {
        self.buttons |= button.mask();
    }

    pub fn clear(&mut self, button: Button) {
        self.buttons &= !button.mask();
    }

    pub fn toggle(&mut self, button: Button) {
        self.buttons ^= button.mask();
    }

    pub fn set_left_trigger(&mut self, value: f32) {
        self.left_trigger = (value * 256_f32) as u8;
    }

    pub fn set_right_trigger(&mut self, value: f32) {
        self.right_trigger = (value * 256_f32) as u8;
    }

    pub fn from_le_bytes(bytes: &[u8; 12]) -> Self {
        let left_stick = StickState::from_bytes(&[bytes[0], bytes[1], bytes[2], bytes[3]]);
        let right_stick = StickState::from_bytes(&[bytes[4], bytes[5], bytes[6], bytes[7]]);
        let buttons = u16::from_le_bytes([bytes[10], bytes[11]]);
        Self {
            left_stick,
            right_stick,
            left_trigger: bytes[8],
            right_trigger: bytes[9],
            buttons,
        }
    }

    pub fn to_le_bytes(&self) -> [u8; 12] {
        let left_stick = self.left_stick.to_bytes();
        let right_stick = self.right_stick.to_bytes();
        let buttons = self.buttons.to_le_bytes();
        [
            left_stick[0],
            left_stick[1],
            left_stick[2],
            left_stick[3],
            right_stick[0],
            right_stick[1],
            right_stick[2],
            right_stick[3],
            self.left_trigger,
            self.right_trigger,
            buttons[0],
            buttons[1],
        ]
    }
}
