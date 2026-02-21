#[allow(unused)]
use core::f64;
#[allow(unused)]
use std::{
    process::exit, sync::{Arc, nonpoison::RwLock}, time::{Duration, Instant}
};

#[allow(unused)]
use vexide::{peripherals::DynamicPeripherals, prelude::Peripherals, time::sleep};

#[allow(unused)]
use crate::{
    autos::{
        auto::{Auto, Autos},
        chassis::{Chassis, Pid},
    }, comp::AutoHandler, conf::Config, cubreg::cubic_regression, log_debug, log_error, log_fatal, log_info, log_warn, telemetry::Telem, tracking::Tracking, util::Drivetrain
};

#[allow(unused)]
#[vexide::test]
async fn logger_test(_peripherals: Peripherals) {
    log_debug!("Hello from log_debug!");
    log_info!("Hello from log_info!");
    log_warn!("Hello from log_warn!");
    log_error!("Hello from log_error!");
    log_fatal!("Hello from log_fatal!");
    sleep(Duration::from_millis(500)).await;
}

#[allow(unused)]
#[vexide::test]
async fn reg_test(_peripherals: Peripherals) {
    log_info!("{:#?}", cubic_regression(vec![-1.0, 0.0, 1.0, 4.0], vec![7.0, 0.0, 1.0, 58.0]));
}

#[allow(unused)]
#[vexide::test]
async fn desaturation_test(_peripherals: Peripherals) {
    let test_val = (1.0, 0.0);
    log_info!("{test_val:?} -> {:?}", crate::autos::auto::desaturate(test_val.clone()));
    assert!(crate::autos::auto::desaturate(test_val) == (1.0, 1.0));
}

#[allow(unused)]
#[vexide::test]
async fn autos_test(peripherals: Peripherals) {
    let conf = Config::load();
    let mut peripherals = DynamicPeripherals::new(peripherals);
    let dt = Drivetrain::new(&conf, &mut peripherals);
    let telem = Telem::new(vec![], vec![]);
    let (mut tracking, pose) = Tracking::new(&mut peripherals, Arc::new(RwLock::new(telem)), Arc::new(RwLock::new(dt)), &conf);
    let linear_pid = Pid::new(8.0, 0.0, 20.0, 0.7, 3.0, 0.25, 400.0, 1.0, 2000.0);
    let angular_pid = Pid::new(8.0, 0.0, 20.0, 0.7, 3.0, 1.0, 400.0, 3.0, 2000.0);
    let heading_pid = Pid::new(2.0, 0.0, 0.0, 0.7, 3.0, 10.0, 1000.0, 30.0, 4000.0);
    let mut chassis = Chassis::new(linear_pid, angular_pid, heading_pid, 0.25, pose.clone());
    let mut comp = crate::setup_autos(AutoHandler::new());
    *comp.selected_auto.write() = Autos::LeftElims;
    let auto = comp.get_auto();
    pose.write().pose.0 = auto.start_pose.0;
    pose.write().pose.1 = auto.start_pose.1;
    pose.write().pose.2 = auto.start_pose.2;
    let (mut l1, mut r1) = (0.0_f64, 0.0_f64);
    let mut last_update_time = Instant::now();
    loop {
        let update = chassis.update(auto);
        log_debug!("{update:?}");
        let dt = last_update_time.elapsed().as_secs_f64();
        last_update_time = Instant::now();
        l1 += update.0 * 3.0 * dt;
        r1 += update.1 * 3.0 * dt;
        tracking.odom_tick(l1, r1);
        let pose = pose.read().pose;
        log_info!("pose: ({:.2}, {:.2}, {:.2}), {}", pose.0, pose.1, pose.2.to_degrees(), auto.current_curve);
        if auto.motion_start.elapsed().as_millis() as f64 >= auto.get_timeout() || auto.exit_state == 2 {
            if auto.current_curve != auto.spline.len() - 1 {
                auto.current_curve += 1
            } else {
                exit(0);
            };
            auto.motion_start = Instant::now();
            auto.exit_state = 0;
            auto.close = false;
        }
        sleep(Duration::from_millis(20)).await;
    }
}
