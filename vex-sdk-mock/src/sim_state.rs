use spin::{RwLock, rwlock::RwLockWriteGuard, rwlock::RwLockReadGuard};

#[derive(Clone)]
pub enum SimDeviceType {
    Motor,
    Imu,
    Optical,
    Rotation,
    Controller
}

#[derive(Clone)]
pub struct SimDevice {
    pub connected: bool,
    pub device_type: SimDeviceType,
    pub data_in: [u64; 64],
    pub data_out: [u64; 64]
}

#[derive(Clone)]
pub struct Battery {
    pub voltage: i32,
    pub current: i32,
    pub temperature: f64,
    pub capacity: f64
}

#[derive(Clone)]
pub struct Display {
    pub writable_area: [u32; 115200]
}

#[derive(Clone)]
pub struct State {
    pub smart_devices: [Option<SimDevice>; 21],
    pub adi_devices: [Option<SimDevice>; 8],
    pub prim_controller: Option<SimDevice>,
    pub part_controller: Option<SimDevice>,
    pub battery: Option<Battery>,
    pub display: Option<Display>
}

impl State {
    const fn default() -> Self {
        Self {
            smart_devices: [const { None }; 21],
            adi_devices: [const { None }; 8],
            prim_controller: None,
            part_controller: None,
            battery: None,
            display: None
        }
    }
}

pub static state_cell: RwLock<State> = RwLock::new(State::default());

pub fn get_mut_state() -> RwLockWriteGuard<'static, State> {
    unsafe { state_cell.write() }
}

pub fn get_state() -> RwLockReadGuard<'static, State> {
    unsafe { state_cell.read() }
}