use spin::RwLock;

pub enum SimDeviceType {
    Motor,
    Imu,
    Optical,
    Rotation,
    Controller
}

pub struct SimDevice {
    connected: bool,
    device_type: SimDeviceType,
    data_in: [u64; 64],
    data_out: [u64; 64]
}

pub struct Battery {
    voltage: i32,
    current: i32,
    temperature: f64,
    capacity: f64
}

pub struct Display {
    writable_area: [u32; 115200]
}

pub struct State {
    smart_devices: [Option<SimDevice>; 21],
    adi_devices: [Option<SimDevice>; 8],
    prim_controller: Option<SimDevice>,
    part_controller: Option<SimDevice>,
    battery: Option<Battery>,
    display: Option<Display>
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

pub(super) static mut state_cell: RwLock<State> = RwLock::new(State::default());