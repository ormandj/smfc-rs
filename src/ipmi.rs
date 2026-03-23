use std::fs::OpenOptions;
use std::os::unix::io::{AsRawFd, RawFd};

use tracing::{debug, trace};

use crate::error::{Error, Result};

// IPMI ioctl constants (from linux/ipmi.h)
const IPMI_IOC_MAGIC: u8 = b'i';
const IPMICTL_SEND_COMMAND: u8 = 13;
const IPMICTL_RECEIVE_MSG_TRUNC: u8 = 11;

// IPMI address types
const IPMI_SYSTEM_INTERFACE_ADDR_TYPE: i32 = 0x0c;
const IPMI_BMC_CHANNEL: i16 = 0x0f;

// Supermicro OEM IPMI commands
const SM_NETFN: u8 = 0x30;
const SM_CMD_FAN_MODE: u8 = 0x45;
const SM_CMD_FAN_DUTY: u8 = 0x70;

/// IPMI fan mode as understood by Supermicro BMC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FanMode {
    Standard = 0,
    Full = 1,
    Optimal = 2,
    Pue = 3,
    HeavyIo = 4,
}

impl FanMode {
    fn from_u8(v: u8) -> Result<Self> {
        match v {
            0 => Ok(FanMode::Standard),
            1 => Ok(FanMode::Full),
            2 => Ok(FanMode::Optimal),
            3 => Ok(FanMode::Pue),
            4 => Ok(FanMode::HeavyIo),
            _ => Err(Error::Ipmi(format!("unknown fan mode: {v}"))),
        }
    }
}

impl std::fmt::Display for FanMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FanMode::Standard => write!(f, "Standard"),
            FanMode::Full => write!(f, "Full"),
            FanMode::Optimal => write!(f, "Optimal"),
            FanMode::Pue => write!(f, "PUE"),
            FanMode::HeavyIo => write!(f, "HeavyIO"),
        }
    }
}

/// IPMI fan zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum IpmiZone {
    Zone0 = 0,
    Zone1 = 1,
}

impl From<&crate::config::ZoneId> for IpmiZone {
    fn from(z: &crate::config::ZoneId) -> Self {
        match z {
            crate::config::ZoneId::Zone0 => IpmiZone::Zone0,
            crate::config::ZoneId::Zone1 => IpmiZone::Zone1,
        }
    }
}

// FFI structures matching linux/ipmi.h

#[repr(C)]
struct IpmiSystemInterfaceAddr {
    addr_type: i32,
    channel: i16,
    lun: u8,
}

#[repr(C)]
struct IpmiMsg {
    netfn: u8,
    cmd: u8,
    data_len: u16,
    data: *mut u8,
}

#[repr(C)]
struct IpmiReq {
    addr: *mut IpmiSystemInterfaceAddr,
    addr_len: u32,
    msgid: i64,
    msg: IpmiMsg,
}

#[repr(C)]
struct IpmiRecv {
    recv_type: i32,
    addr: *mut IpmiSystemInterfaceAddr,
    addr_len: u32,
    msgid: i64,
    msg: IpmiMsg,
}

// Generate ioctl request/response function numbers.
// Linux IPMI ioctls famously use backwards direction macros:
// IPMICTL_SEND_COMMAND is _IOR (not _IOW) in the kernel headers.
// We must match the kernel's ioctl numbers exactly.
nix::ioctl_read!(
    ipmi_send_command,
    IPMI_IOC_MAGIC,
    IPMICTL_SEND_COMMAND,
    IpmiReq
);

nix::ioctl_readwrite!(
    ipmi_receive_msg,
    IPMI_IOC_MAGIC,
    IPMICTL_RECEIVE_MSG_TRUNC,
    IpmiRecv
);

/// Handle to an open IPMI device.
pub struct IpmiDevice {
    fd: RawFd,
    // Keep the file alive so fd stays valid
    _file: std::fs::File,
}

impl IpmiDevice {
    /// Open the IPMI device at the given path.
    pub fn open(path: &str) -> Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|e| Error::Ipmi(format!("failed to open {path}: {e}")))?;

        Ok(Self {
            fd: file.as_raw_fd(),
            _file: file,
        })
    }

    /// Send a raw IPMI command and return the response data.
    pub fn raw_command(&self, netfn: u8, cmd: u8, data: &[u8]) -> Result<Vec<u8>> {
        trace!(
            netfn = format!("0x{netfn:02x}"),
            cmd = format!("0x{cmd:02x}"),
            data_len = data.len(),
            "sending IPMI command"
        );

        let mut addr = IpmiSystemInterfaceAddr {
            addr_type: IPMI_SYSTEM_INTERFACE_ADDR_TYPE,
            channel: IPMI_BMC_CHANNEL,
            lun: 0,
        };

        let mut send_data = data.to_vec();

        let mut req = IpmiReq {
            addr: &mut addr as *mut _,
            addr_len: std::mem::size_of::<IpmiSystemInterfaceAddr>() as u32,
            msgid: 1,
            msg: IpmiMsg {
                netfn,
                cmd,
                data_len: send_data.len() as u16,
                data: if send_data.is_empty() {
                    std::ptr::null_mut()
                } else {
                    send_data.as_mut_ptr()
                },
            },
        };

        // Send
        unsafe {
            ipmi_send_command(self.fd, &mut req)
                .map_err(|e| Error::Ipmi(format!("ioctl send failed: {e}")))?;
        }

        // Wait for response with poll
        let poll_fd = nix::poll::PollFd::new(
            unsafe { std::os::unix::io::BorrowedFd::borrow_raw(self.fd) },
            nix::poll::PollFlags::POLLIN,
        );
        nix::poll::poll(&mut [poll_fd], nix::poll::PollTimeout::from(5000u16))
            .map_err(|e| Error::Ipmi(format!("poll failed: {e}")))?;

        // Receive
        let mut recv_addr = IpmiSystemInterfaceAddr {
            addr_type: 0,
            channel: 0,
            lun: 0,
        };

        let mut recv_buf = vec![0u8; 256];

        let mut recv = IpmiRecv {
            recv_type: 0,
            addr: &mut recv_addr as *mut _,
            addr_len: std::mem::size_of::<IpmiSystemInterfaceAddr>() as u32,
            msgid: 0,
            msg: IpmiMsg {
                netfn: 0,
                cmd: 0,
                data_len: recv_buf.len() as u16,
                data: recv_buf.as_mut_ptr(),
            },
        };

        unsafe {
            ipmi_receive_msg(self.fd, &mut recv)
                .map_err(|e| Error::Ipmi(format!("ioctl receive failed: {e}")))?;
        }

        let resp_len = recv.msg.data_len as usize;
        if resp_len == 0 {
            return Err(Error::Ipmi("empty IPMI response".to_string()));
        }

        // First byte of response data is the completion code
        let cc = recv_buf[0];
        if cc != 0x00 {
            return Err(Error::IpmiCompletion {
                netfn: recv.msg.netfn,
                cmd: recv.msg.cmd,
                cc,
            });
        }

        // Return the response data after the completion code
        let response = recv_buf[1..resp_len].to_vec();
        trace!(response_len = response.len(), "IPMI command completed");
        Ok(response)
    }
}

/// Get the current BMC fan mode.
pub fn get_fan_mode(dev: &IpmiDevice) -> Result<FanMode> {
    let resp = dev.raw_command(SM_NETFN, SM_CMD_FAN_MODE, &[0x00])?;
    if resp.is_empty() {
        return Err(Error::Ipmi("empty fan mode response".to_string()));
    }
    let mode = FanMode::from_u8(resp[0])?;
    debug!(mode = %mode, "read fan mode");
    Ok(mode)
}

/// Set the BMC fan mode.
pub fn set_fan_mode(dev: &IpmiDevice, mode: FanMode) -> Result<()> {
    debug!(mode = %mode, "setting fan mode");
    dev.raw_command(SM_NETFN, SM_CMD_FAN_MODE, &[0x01, mode as u8])?;
    Ok(())
}

/// Set the fan duty cycle for a zone (0-100%).
pub fn set_fan_duty(dev: &IpmiDevice, zone: IpmiZone, duty: u8) -> Result<()> {
    let duty = duty.min(100);
    debug!(zone = ?zone, duty, "setting fan duty");
    dev.raw_command(SM_NETFN, SM_CMD_FAN_DUTY, &[0x66, 0x01, zone as u8, duty])?;
    Ok(())
}
