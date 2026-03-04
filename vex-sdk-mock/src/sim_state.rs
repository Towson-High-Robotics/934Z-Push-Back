use core::{ffi::c_void, ptr::null_mut};

use spin::{RwLock, rwlock::RwLockWriteGuard, rwlock::RwLockReadGuard};
use vex_sdk::{V5_Device, V5_DeviceT, V5_DeviceType};

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

#[repr(C)]
pub struct State {
    pub smart_devices: [SimDevice; 22],
    pub test: [SmartPort; 22],
    pub device_ptrs: [*mut *mut c_void; 22],
    pub adi_devices: [SimDevice; 8],
    pub prim_controller: SimDevice,
    pub part_controller: SimDevice,
    pub battery: Battery,
    pub display: Display
}

#[derive(Debug, Default)]
pub struct SmartPort {
    index: usize
}

impl State {
    const fn default() -> Self {
        Self {
            smart_devices: [const { SimDevice::new() }; 22],
            test: [ const { SmartPort { index: 0 } }; 22],
            device_ptrs: [const { null_mut() }; 22],
            adi_devices: [const { SimDevice::new() }; 8],
            prim_controller: SimDevice::new(),
            part_controller: SimDevice::new(),
            battery: Battery::new(),
            display: Display::new()
        }
    }

    pub fn generate_ptrs(&mut self) {
        self.smart_devices.iter_mut().enumerate().for_each(|mut element| {
            self.device_ptrs[element.0] = &mut SmartPort { index: element.0 } as *mut _ as *mut *mut c_void;
        });
    }
}

pub static mut state_cell: State = State::default();