use alloc::string::String;

#[derive(Debug, serde::Deserialize)]
pub struct OpenMeteoResponse {
    pub daily: OpenMeteoDaily,
    pub current: OpenMeteoCurrent,
    pub hourly: OpenMeteoHourly,
}

#[derive(Debug, serde::Deserialize)]
pub struct OpenMeteoDaily {
    pub weather_code: [f32; 1],
    pub temperature_2m_max: [f32; 1],
    pub temperature_2m_min: [f32; 1],
}

#[derive(Debug, serde::Deserialize)]
pub struct OpenMeteoCurrent {
    pub temperature_2m: f32,
    pub time: String,
}

#[derive(Debug, serde::Deserialize)]
pub struct OpenMeteoHourly {
    pub precipitation_probability: [f32; 24],
}
