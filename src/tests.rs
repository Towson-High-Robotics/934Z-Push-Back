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
        auto::{Auto, Autos, Action},
        chassis::{Chassis, Pid},
    },
    comp::AutoHandler,
    conf::Config,
    cubreg::cubic_regression,
    log_debug,
    log_error,
    log_fatal,
    log_info,
    log_warn,
    telemetry::Telem,
    tracking::{Tracking, TrackingSensors},
    util::Drivetrain
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

    let dt = Arc::new(RwLock::new(Drivetrain::new(&conf, &mut peripherals)));
    let telem = Arc::new(RwLock::new(Telem::new(vec![], vec![])));

    let sensors = TrackingSensors::new(&mut peripherals, [11, 14, 15, 17, 18, 19], [0.0, 0.0, 2.0, 2.0, 2.0], [180.0, 0.0, 90.0], [false, false]);
    let tracking = Arc::new(RwLock::new(Tracking::new(sensors, telem.clone(), dt.clone())));
    let linear_pid = Pid::new(8.0, 0.0, 20.0, 0.7, 3.0, 0.25, 400.0, 1.0, 2000.0);
    let angular_pid = Pid::new(8.0, 0.0, 20.0, 0.7, 3.0, 1.0, 400.0, 3.0, 2000.0);
    let mut chassis = Chassis::new(linear_pid, angular_pid, 0.25, tracking.clone());

    let mut comp = crate::setup_autos(AutoHandler::new());
    *comp.selected_auto.write() = Autos::None;

    let auto = comp.get_auto();
    chassis.calibrate(auto.start_pose).await;
    tracking.write().pose = auto.start_pose;
    let (mut l1, mut r1) = (0.0_f64, 0.0_f64);
    let mut update;

    let mut last_update_time = Instant::now();
    let runtime = Instant::now();

    loop {
        if runtime.elapsed().as_secs_f64() >= 15.00 { exit(0); }
        
        let dt = last_update_time.elapsed().as_secs_f64();
        update = if auto.motion_start.elapsed().as_secs_f64() * 1000.0 >= auto.get_timeout() || auto.exit_state == 2 {
            auto.motion_start = Instant::now();
            auto.exit_state = 3;
            (0.0, 0.0)
        } else if auto.motion_start.elapsed().as_secs_f64() * 1000.0 >= auto.get_wait() && auto.exit_state == 3 {
            if auto.current_curve != auto.spline.len() - 1 {
                auto.current_curve += 1;
                auto.motion_start = Instant::now();
                auto.exit_state = 0;
                auto.close = false;
            }
            (0.0, 0.0)
        } else if auto.exit_state == 3 {
            (0.0, 0.0)
        } else {
            let updatet = chassis.update(auto);
            //log_debug!("{updatet:?}");
            updatet
        };

        if auto.current_action + 1 < auto.actions.len() {
            for i in auto.current_action..(auto.actions.len() - 1) {
                let action = &auto.actions[i];
                if (action.1 - (auto.current_curve as f64 + auto.curve_t.clamp(0.0, 1.0))).abs() < 0.025 {
                    match action.0 {
                        Action::ToggleMatchload => log_info!("Toggled Matchload"),
                        Action::ToggleDescore => log_info!("Toggled Descore/Hood"),
                        Action::SpinIntake(v) => log_info!("Span Intake at {v:.1} mV"),
                        Action::StopIntake => log_info!("Stopped Intake"),
                        Action::SpinIndexer(v) => log_info!("Span Indexer at {v:.1} mV"),
                        Action::StopIndexer => log_info!("Stopped Indexer"),
                        Action::ResetPose(x, y, theta) => chassis.set_pose((x, y, theta)),
                        Action::DistanceReset(s) => log_info!("Distance Reset on Sensor {s}"),
                    };
                } else {
                    break;
                }
            }
        }
        
        l1 += update.0 * 20.0 * dt;
        r1 += update.1 * 20.0 * dt;
        tracking.write().odom_tick(l1, r1);

        last_update_time = Instant::now();
        //let pose = tracking.read().pose;
        //log_info!("pose: ({:.2}, {:.2}, {:.2}), {}", pose.0, pose.1, pose.2.to_degrees(), auto.exit_state);

        sleep(Duration::from_millis(10)).await;
    }
}
