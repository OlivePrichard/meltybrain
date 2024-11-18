use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};

use crate::shared_code::controller::ControllerState;

#[embassy_executor::task]
pub async fn control_logic(controllers: &'static Mutex<NoopRawMutex, (ControllerState, ControllerState)>, armed: &'static Mutex<NoopRawMutex, bool>) -> ! {
    loop {
        yield_now().await;
    }
}