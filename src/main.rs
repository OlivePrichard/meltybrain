#![no_std]
#![no_main]

mod control_logic;
mod logging;
mod networking;
mod shared_code;
mod watchdog;

use control_logic::control_logic;
use embassy_executor::Spawner;
use embassy_net::{Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::Io,
    ledc::{channel, timer, Ledc, LowSpeed},
    prelude::*,
    rng::Rng,
    timer::timg::TimerGroup,
};
// use esp_println::println;
use esp_wifi::{
    init,
    wifi::{
        AccessPointConfiguration, Configuration, WifiApDevice, WifiController, WifiDevice,
        WifiEvent, WifiState,
    },
    EspWifiInitFor,
};

use logging::log;
use networking::handle_networking;
use shared_code::controller::ControllerState;
use watchdog::Watchdog;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// gateway ip is 192.168.2.1
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) -> ! {
    // println!("start connection task");
    // println!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::ApStarted => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::ApStop).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::AccessPoint(AccessPointConfiguration {
                ssid: "esp-wifi".try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            // println!("Starting wifi");
            controller.start().await.unwrap();
            // println!("Wifi started!");
            log!(WifiStarted);
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static, WifiApDevice>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn watchdog_task(
    watchdog: &'static Watchdog,
    armed: &'static Mutex<NoopRawMutex, bool>,
) -> ! {
    let delay = Duration::from_hz(50);
    watchdog.wait_for_start(delay).await;
    {
        let mut lock = armed.lock().await;
        *lock = true;
    }

    loop {
        watchdog.run().await;
        log!(WatchdogTimeout);
        {
            let mut lock = armed.lock().await;
            *lock = false;
        }
        while !watchdog.is_fed().await {
            Timer::after(delay).await;
        }
        log!(ConnectionRestored);
        {
            let mut lock = armed.lock().await;
            *lock = true;
        }
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    log!(Initializing);

    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let left_motor_pin = io.pins.gpio21;
    let right_motor_pin = io.pins.gpio5;

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

    let lstimer0 = mk_static!(
        timer::Timer<LowSpeed>,
        ledc.get_timer::<LowSpeed>(timer::Number::Timer0)
    );
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 4.kHz(),
        })
        .unwrap();

    let mut left_motor = ledc.get_channel(channel::Number::Channel0, left_motor_pin);
    left_motor
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    let mut right_motor = ledc.get_channel(channel::Number::Channel0, right_motor_pin);
    right_motor
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let init = &*mk_static!(
        esp_wifi::EspWifiInitialization,
        init(
            EspWifiInitFor::Wifi,
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiApDevice).unwrap();

    use esp_hal::timer::systimer::{SystemTimer, Target};
    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    esp_hal_embassy::init(systimer.alarm0);

    log!(Initialized);

    let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 2, 1), 24),
        gateway: Some(Ipv4Address::from_bytes(&[192, 168, 2, 1])),
        dns_servers: Default::default(),
    });

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiApDevice>>,
        Stack::new(
            wifi_interface,
            config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );

    let connection_watchdog = &*mk_static!(Watchdog, Watchdog::new(Duration::from_secs(2)));
    let controller_data = &*mk_static!(
        Mutex<NoopRawMutex, (ControllerState, ControllerState)>,
        Mutex::new((ControllerState::default(), ControllerState::default()))
    );
    let armed = &*mk_static!(
        Mutex<NoopRawMutex, bool>,
        Mutex::new(false)
    );

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(&stack)).ok();
    spawner
        .spawn(handle_networking(
            stack,
            controller_data,
            connection_watchdog,
        ))
        .ok();
    spawner
        .spawn(watchdog_task(connection_watchdog, armed))
        .ok();
    spawner
        .spawn(control_logic(
            controller_data,
            armed,
            left_motor,
            right_motor,
        ))
        .ok();

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
