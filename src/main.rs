#![no_main]
#![no_std]
#![feature(slice_as_array)]
#![feature(const_type_id)]

extern crate alloc;

use vexide::prelude::*;

pub mod util;
use util::*;

pub mod tracking;
use tracking::*;

pub mod conf;

impl Compete for Robot {
    async fn connected(&mut self) { self.connected = true; }
    async fn disconnected(&mut self) { self.connected = false; }
    async fn disabled(&mut self) {}
    async fn driver(&mut self) {
        if self.connected {
            
        } else {

        }
    }
    async fn autonomous(&mut self) {}
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let dyn_peripherals = DynamicPeripherals::new(peripherals);
    let mut robot = Robot::new(dyn_peripherals);
    robot.update_from_conf();
    robot.compete().await;
}