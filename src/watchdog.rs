use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};

struct WatchdogInner {
    wakeup: Instant,
    timeout: Duration,
    started: bool,
}

pub struct Watchdog {
    inner: Mutex<CriticalSectionRawMutex, WatchdogInner>,
}

impl Watchdog {
    pub fn new(timeout: Duration) -> Self {
        Self {
            inner: Mutex::new(WatchdogInner {
                wakeup: Instant::now(),
                timeout,
                started: false,
            }),
        }
    }

    pub async fn feed(&self) {
        let mut watchdog = self.inner.lock().await;
        watchdog.wakeup = Instant::now() + watchdog.timeout;
    }

    pub async fn start(&self) {
        let mut watchdog = self.inner.lock().await;
        watchdog.started = true;
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

    pub async fn is_fed(&self) -> bool {
        let watchdog = self.inner.lock().await;
        Instant::now() < watchdog.wakeup
    }

    pub async fn wait_for_start(&self, timeout: Duration) {
        loop {
            let started = {
                let watchdog = self.inner.lock().await;
                watchdog.started
            };
            if started {
                return;
            }
            Timer::after(timeout).await;
        }
    }
}
