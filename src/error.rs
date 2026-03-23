use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("config error: {0}")]
    Config(String),

    #[error("TOML parse error: {0}")]
    Toml(#[from] toml::de::Error),

    #[error("IPMI error: {0}")]
    Ipmi(String),

    #[error("IPMI completion code error: netfn=0x{netfn:02x} cmd=0x{cmd:02x} cc=0x{cc:02x}")]
    IpmiCompletion { netfn: u8, cmd: u8, cc: u8 },

    #[error("hwmon error: {0}")]
    Hwmon(String),
}

pub type Result<T> = std::result::Result<T, Error>;
