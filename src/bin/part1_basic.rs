#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(try_blocks)]

use alloc::format;
use anyhow::anyhow;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{DhcpConfig, StaticConfigV4};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pubsub::{self, PubSubChannel};
use embassy_time::{Duration, Ticker, Timer};
use embedded_graphics::mono_font::ascii::FONT_9X15_BOLD;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::prelude::{Point, Size};
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use embedded_layout::align::{horizontal, vertical, Align};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Input, Pin};
use esp_hal::peripherals::{GPIO32, GPIO4, GPIO5, I2C0, TOUCH};
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::touch::{Touch, TouchConfig, TouchPad};
use esp_println::println;
use esp_wifi::wifi::{ClientConfiguration, Configuration, WifiController, WifiDevice, WifiState};
use esp_wifi::EspWifiController;
use log::{debug, error, info};
use reqwless::client::{HttpClient, TlsConfig};
use reqwless::request::Method;
use ssd1306::mode::DisplayConfigAsync;
use ssd1306::prelude::{Brightness, I2CInterface};
use ssd1306::size::DisplaySize128x64;
use static_cell::{make_static, StaticCell};
use time::format_description::well_known::Iso8601;
use time::PrimitiveDateTime;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[derive(Debug, Clone, Copy)]
enum AppEvent {
    NetworkEnabled,
    NetworkDisabled,
    WeatherChanged { temp: f32 },
    TimeChanged(PrimitiveDateTime),
    ButtonPressed,
}

type PubSub = PubSubChannel<NoopRawMutex, AppEvent, 32, 16, 16>;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.4.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG); // change to mut
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = esp_wifi::init(timer1.timer0, rng, peripherals.RADIO_CLK)
        .expect("Failed to initialize WIFI/BLE controller");
    let (wifi_controller, interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller"); // remove _

    let mut net_stack_resources = embassy_net::StackResources::<4>::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let (net_stack, mut sta_runner) = embassy_net::new(
        interfaces.sta,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        &mut net_stack_resources,
        seed,
    );

    static PUB_SUB: StaticCell<PubSub> = StaticCell::new();
    let pub_sub = &*PUB_SUB.init(PubSubChannel::new());

    spawner.must_spawn(screen_renderer_task(
        &pub_sub,
        peripherals.GPIO5,
        peripherals.GPIO4,
        peripherals.I2C0,
    ));

    embassy_futures::join::join3(
        sta_runner.run(),
        wifi_connection("SSID", "PASS", wifi_controller),
        weather_updater(&pub_sub, net_stack),
    )
    .await;
}

#[embassy_executor::task]
async fn screen_renderer_task(
    pubsub: &'static PubSub,
    sda_pin: GPIO5<'static>,
    scl_pin: GPIO4<'static>,
    i2c_per: I2C0<'static>,
) {
    let mut i2c = esp_hal::i2c::master::I2c::new(
        i2c_per,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400u32)),
    )
    .unwrap()
    .with_sda(sda_pin)
    .with_scl(scl_pin)
    .into_async();
    let i2c_graphics = I2CInterface::new(&mut i2c, 0x3C, 0x40);
    let mut display = ssd1306::Ssd1306Async::new(
        i2c_graphics,
        DisplaySize128x64,
        ssd1306::prelude::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().await.unwrap();
    display.set_brightness(Brightness::BRIGHTEST).await.unwrap();

    let screen_area = Rectangle::new(Point::zero(), Size::new(128, 64));
    let mut date = PrimitiveDateTime::MIN;
    let mut sub = pubsub.dyn_subscriber().unwrap();

    loop {
        match sub.next_message_pure().await {
            AppEvent::WeatherChanged { .. } => {}
            AppEvent::TimeChanged(d) => date = d,
            AppEvent::ButtonPressed => {}
            _ => {}
        }

        display.clear_buffer();

        let time_str = format!(
            "{:0>2}:{:0>2}:{:0>2}",
            date.hour(),
            date.minute(),
            date.second()
        );
        let time_text = Text::with_text_style(
            time_str.as_str(),
            Point::zero(),
            MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On),
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Middle)
                .build(),
        )
        .align_to(&screen_area, horizontal::Center, vertical::Top);
        time_text.draw(&mut display).unwrap();

        display.flush().await.unwrap();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

async fn weather_updater(pubsub: &'static PubSub, net_stack: embassy_net::Stack<'_>) {
    const RX_SIZE: usize = 4096;
    const TX_SIZE: usize = 4096;
    let dns = DnsSocket::new(net_stack);
    let tcp_state = TcpClientState::<1, TX_SIZE, RX_SIZE>::new();
    let tcp = TcpClient::new(net_stack, &tcp_state);

    let url = "http://api.open-meteo.com/v1/forecast?latitude=55.7522&longitude=37.6156&hourly=temperature_2m&current=temperature_2m&timezone=Europe%2FMoscow&forecast_days=1";
    let mut client = HttpClient::new(&tcp, &dns);
    let publisher = pubsub.immediate_publisher();
    let mut http_resp_buffer = [0u8; RX_SIZE];

    loop {
        if net_stack.is_link_up() {
            let date: anyhow::Result<PrimitiveDateTime> = try {
                let mut http_request = client
                    .request(Method::GET, url)
                    .await
                    .map_err(|e| anyhow!("error request {:?}", e))?;

                let response = http_request
                    .send(&mut http_resp_buffer)
                    .await
                    .map_err(|e| anyhow!("error send {:?}", e))?;

                let res = response
                    .body()
                    .read_to_end()
                    .await
                    .map_err(|e| anyhow!("error body read {:?}", e))?;

                let json = serde_json::from_slice::<serde_json::Value>(&res)
                    .map_err(|e| anyhow!("json syntax error {:?}", e))?;

                let date_raw = json["current"]["time"]
                    .as_str()
                    .ok_or(anyhow!("fail to get json 'current.time' field"))?;

                let date = PrimitiveDateTime::parse(date_raw, &Iso8601::DEFAULT)
                    .map_err(|e| anyhow!("fail to parse date {:?}", e))?;

                date
            };
            if let Ok(date) = date {
                publisher.publish_immediate(AppEvent::TimeChanged(date));
            }
            debug!("http response {:?}", date);
        }
        Timer::after(Duration::from_millis(5000)).await;
    }
}

async fn wifi_connection<'a>(ssid: &str, password: &str, mut controller: WifiController<'a>) {
    loop {
        let wifi_state = esp_wifi::wifi::sta_state();
        if wifi_state == WifiState::Invalid {
            controller
                .set_configuration(&Configuration::Client(ClientConfiguration::default()))
                .unwrap();
            controller.start_async().await.unwrap();
        }
        if wifi_state == WifiState::StaStarted {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: ssid.try_into().unwrap(),
                password: password.try_into().unwrap(),
                ..Default::default()
            });
            let _ = controller
                .set_configuration(&client_config)
                .inspect(|e| error!("{:?}", e));
            let _ = controller.connect_async().await;
        }
        Timer::after(Duration::from_millis(5000)).await;
    }
}
