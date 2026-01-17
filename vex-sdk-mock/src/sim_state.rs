use spin::{RwLock, rwlock::RwLockWriteGuard, rwlock::RwLockReadGuard};
use vex_sdk::V5_DeviceType;

pub struct SimDevice {
    pub connected: bool,
    pub device_type: V5_DeviceType,
    pub data_in: [u64; 64],
    pub data_out: [u64; 64],
    pub pos: (f64, f64, f64),
}

impl SimDevice {
    pub const fn new() -> Self {
        Self {
            connected: false,
            device_type: V5_DeviceType::kDeviceTypeNoSensor,
            data_in: [0; 64],
            data_out: [0; 64],
            pos: (0.0, 0.0, 0.0)
        }
    }
}

pub struct SimDeviceWrapper {
    device: RwLock<SimDevice>
}

impl SimDeviceWrapper {
    pub const fn new(raw: SimDevice) -> Self {
        Self { device: RwLock::new(raw) }
    }

    pub fn get(&mut self) -> RwLockWriteGuard<'_, SimDevice> {
        self.device.write()
    }
}

pub struct Battery {
    pub voltage: i32,
    pub current: i32,
    pub temperature: f64,
    pub capacity: f64
}

impl Battery {
    pub const fn new() -> Battery {
        Self {
            voltage: 0,
            current: 0,
            temperature: 25.0,
            capacity: 1.0,
        }
    }
}

pub struct Display {
    pub writable_area: [u32; 115200]
}

impl Display {
    pub const fn new() -> Self {
        Self {
            writable_area: [0; 115200]
        }
    }
}

pub struct State {
    pub smart_devices: [SimDeviceWrapper; 22],
    pub adi_devices: [SimDeviceWrapper; 8],
    pub prim_controller: SimDeviceWrapper,
    pub part_controller: SimDeviceWrapper,
    pub battery: RwLock<Battery>,
    pub display: RwLock<Display>
}

impl State {
    const fn default() -> Self {
        Self {
            smart_devices: [const { SimDeviceWrapper::new(SimDevice::new()) }; 22],
            adi_devices: [const { SimDeviceWrapper::new(SimDevice::new()) }; 8],
            prim_controller: SimDeviceWrapper::new(SimDevice::new()),
            part_controller: SimDeviceWrapper::new(SimDevice::new()),
            battery: RwLock::new(Battery::new()),
            display: RwLock::new(Display::new())
        }
    }
}

pub static mut state_cell: State = State::default();