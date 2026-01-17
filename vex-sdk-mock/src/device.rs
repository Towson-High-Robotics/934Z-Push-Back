//! V5 Smart Devices

use core::ffi::{c_double, c_int, c_void};
use spin::{RwLock, rwlock::RwLockWriteGuard};
use vex_sdk::V5_Device;
pub use vex_sdk::{V5_DeviceT, V5_DeviceType};

use crate::{SimDeviceWrapper, state_cell};

#[unsafe(no_mangle)]
pub extern "C" fn vexDevicesGetNumber() -> u32 {
    23
}
#[unsafe(no_mangle)]
pub extern "C" fn vexDevicesGetNumberByType(device_type: V5_DeviceType) -> u32 {
    Default::default()
}
#[unsafe(no_mangle)]
pub extern "C" fn vexDevicesGet() -> V5_DeviceT {
    core::ptr::null_mut()
}
#[unsafe(no_mangle)]
pub extern "C" fn vexDeviceGetByIndex(index: u32) -> V5_DeviceT {
    unsafe {
        let mut device = &mut state_cell.smart_devices[index as usize];
        let mut device_ptr = (device as *mut _ as V5_Device);
        &mut device_ptr as V5_DeviceT
    }
}
#[unsafe(no_mangle)]
pub extern "C" fn vexDeviceFlagsGetByIndex(index: u32) -> u32 {
    Default::default()
}
#[unsafe(no_mangle)]
pub extern "C" fn vexDeviceGetStatus(devices: *mut V5_DeviceType) -> i32 {
    -1
}
#[unsafe(no_mangle)]
pub extern "C" fn vexDeviceGetTimestamp(device: V5_DeviceT) -> u32 {
    Default::default()
}
#[unsafe(no_mangle)]
pub extern "C" fn vexDeviceGenericValueGet(device: V5_DeviceT) -> c_double {
    Default::default()
}
#[unsafe(no_mangle)]
pub extern "C" fn vexDeviceTypeGetByIndex(index: u32) -> V5_DeviceType {
    unsafe { (&mut *(*vexDeviceGetByIndex(index) as *mut SimDeviceWrapper)).get().device_type.clone() }
}
#[unsafe(no_mangle)]
pub extern "C" fn vexDeviceButtonStateGet() -> c_int {
    Default::default()
}
