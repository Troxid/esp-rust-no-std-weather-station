#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(try_blocks)]

extern crate alloc;
use alloc::format;
use anyhow::anyhow;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::DhcpConfig;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Timer};
use embedded_graphics::mono_font::ascii::{
    FONT_5X7, FONT_5X8, FONT_6X13_BOLD, FONT_7X13, FONT_9X15_BOLD,
};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::prelude::{Point, Size};
use embedded_graphics::primitives::{Rectangle};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use embedded_layout::align::{horizontal, vertical, Align};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::{GPIO4, GPIO5, I2C0};
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_rust_no_std_weather_station::draw_plot::TimePlot;
use esp_rust_no_std_weather_station::dto::OpenMeteoResponse;
use esp_wifi::wifi::{ClientConfiguration, Configuration, WifiController, WifiState};
use log::{debug, error, info};
use reqwless::client::{HttpClient, TlsConfig};
use reqwless::request::Method;
use ssd1306::mode::DisplayConfigAsync;
use ssd1306::prelude::{Brightness, I2CInterface};
use ssd1306::size::DisplaySize128x64;
use static_cell::{StaticCell};
use time::format_description::well_known::Iso8601;
use time::PrimitiveDateTime;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[derive(Debug, Clone, Copy)]
enum AppEvent {
    NetworkEnabled,
    NetworkDisabled,
    WeatherChanged {
        current: f32,
        max: f32,
        min: f32,
        rain_probability: [f32; 24],
    },
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

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = esp_wifi::init(timer1.timer0, rng, peripherals.RADIO_CLK)
        .expect("Failed to initialize WIFI/BLE controller");
    let (wifi_controller, interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");

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
        weather_updater(&pub_sub, net_stack, rng),
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

    let mut is_first_page = true;
    let mut date = PrimitiveDateTime::MIN;
    let mut min_t: f32 = -20.0;
    let mut max_t: f32 = 20.0;
    let mut cur_t: f32 = 10.0;
    let mut rain_p: [f32; 24] = [100.0; 24];
    let mut sub = pubsub.dyn_subscriber().unwrap();

    let mut plot = TimePlot::new();
    plot.bar_height = 20;
    plot.bar_width = 4;
    plot.bar_margin = 1;
    plot = plot.align_to(&screen_area, horizontal::Center, vertical::Bottom);

    loop {
        match sub.next_message_pure().await {
            AppEvent::WeatherChanged {
                current,
                max,
                min,
                rain_probability,
            } => {
                cur_t = current;
                max_t = max;
                min_t = min;
                rain_p = rain_probability;
            }
            AppEvent::TimeChanged(d) => date = d,
            AppEvent::ButtonPressed => is_first_page = !is_first_page,
            _ => {}
        }

        display.clear_buffer();

        // plot
        let plot_label = Text::with_text_style(
            "Rain %",
            Point::zero(),
            MonoTextStyle::new(&FONT_6X13_BOLD, BinaryColor::On),
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Middle)
                .build(),
        )
        .align_to(&plot, horizontal::Left, vertical::BottomToTop)
        .translate(Point::new(0, -5));
        plot_label.draw(&mut display).unwrap();
        plot.pointer = date.hour();
        plot.values = rain_p;
        plot.draw(&mut display).unwrap();

        // time and date
        let time_str = format!("{:0>2}:{:0>2}", date.hour(), date.minute());
        let time_text = Text::with_text_style(
            time_str.as_str(),
            Point::zero(),
            MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On),
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Middle)
                .build(),
        )
        .align_to(&screen_area, horizontal::Left, vertical::Top);
        time_text.draw(&mut display).unwrap();

        let date_str = format!(
            "{:0>2}:{:0>2}:{:0>2}",
            date.day(),
            u8::from(date.month()),
            date.year(),
        );
        let date_text = Text::with_text_style(
            date_str.as_str(),
            Point::zero(),
            MonoTextStyle::new(&FONT_5X8, BinaryColor::On),
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Middle)
                .build(),
        )
        .align_to(&time_text, horizontal::Center, vertical::TopToBottom)
        .align_to(&screen_area, horizontal::Left, vertical::NoAlignment);
        date_text.draw(&mut display).unwrap();

        // Max, min and current temperature
        let weather_area = Rectangle::new(Point::zero(), Size::new(65, 25)).align_to(
            &screen_area,
            horizontal::Right,
            vertical::Top,
        );

        Text::new(
            format!("{:+}", cur_t).as_str(),
            Point::zero(),
            MonoTextStyle::new(&FONT_7X13, BinaryColor::On),
        )
        .align_to(&weather_area, horizontal::Left, vertical::Center)
        .draw(&mut display)
        .unwrap();

        Text::new(
            format!("{:+}", min_t).as_str(),
            Point::zero(),
            MonoTextStyle::new(&FONT_5X7, BinaryColor::On),
        )
        .align_to(&weather_area, horizontal::Right, vertical::Bottom)
        .draw(&mut display)
        .unwrap();

        Text::new(
            format!("{:+}", max_t).as_str(),
            Point::zero(),
            MonoTextStyle::new(&FONT_5X7, BinaryColor::On),
        )
        .align_to(&weather_area, horizontal::Right, vertical::Top)
        .draw(&mut display)
        .unwrap();

        // flush
        display.flush().await.unwrap();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

async fn weather_updater(pubsub: &'static PubSub, net_stack: embassy_net::Stack<'_>, mut rng: Rng) {
    const RX_SIZE: usize = 4096;
    const TX_SIZE: usize = 4096;
    let mut rx_buffer = [0; RX_SIZE];
    let mut tx_buffer = [0; TX_SIZE];
    let dns = DnsSocket::new(net_stack);
    let tcp_state = TcpClientState::<1, TX_SIZE, RX_SIZE>::new();
    let tcp = TcpClient::new(net_stack, &tcp_state);

    let tls_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
    let tls = TlsConfig::new(
        tls_seed,
        &mut rx_buffer,
        &mut tx_buffer,
        reqwless::client::TlsVerify::None,
    );

    let url = "http://api.open-meteo.com/v1/forecast?\
            latitude=55.7522&longitude=37.6156&\
            current=temperature_2m&hourly=precipitation_probability&\
            daily=weather_code,temperature_2m_max,temperature_2m_min,sunrise,sunset&\
            timezone=Europe%2FMoscow&forecast_days=1";
    let mut client = HttpClient::new_with_tls(&tcp, &dns, tls);
    let publisher = pubsub.immediate_publisher();
    let mut http_resp_buffer = [0u8; 2048];

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

                let json = serde_json::from_slice::<OpenMeteoResponse>(&res)
                    .map_err(|e| anyhow!("json syntax error {:?}", e))?;

                let date = PrimitiveDateTime::parse(&json.current.time, &Iso8601::DEFAULT)
                    .map_err(|e| anyhow!("fail to parse date {:?}", e))?;

                publisher.publish_immediate(AppEvent::TimeChanged(date));
                publisher.publish_immediate(AppEvent::WeatherChanged {
                    current: json.current.temperature_2m,
                    max: json.daily.temperature_2m_max[0],
                    min: json.daily.temperature_2m_min[0],
                    rain_probability: json.hourly.precipitation_probability,
                });

                date
            };

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
