use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};

struct WatchdogInner {
    wakeup: Instant,
    timeout: Duration,
}

pub struct Watchdog {
    inner: Mutex<NoopRawMutex, WatchdogInner>,
}

impl Watchdog {
    pub fn new(timeout: Duration) -> Self {
        Self {
            inner: Mutex::new(WatchdogInner {
                wakeup: Instant::now(),
                timeout,
            }),
        }
    }

    pub async fn feed(&self) {
        let mut watchdog = self.inner.lock().await;
        watchdog.wakeup = Instant::now() + watchdog.timeout;
    }

    // This function will only return once the watchdog gets angry
    pub async fn run(&self) {
        loop {
            let wakeup = {
                let watchdog = self.inner.lock().await;
                watchdog.wakeup
            };

            let now = Instant::now();
            if now >= wakeup {
                return;
            }
            Timer::at(wakeup).await;
        }
    }
}
