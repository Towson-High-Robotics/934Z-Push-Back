use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use dear_imgui_rs::ConfigFlags;
use dear_imgui_sdl3::{enable_native_ime_ui, init_for_opengl, new_frame, process_sys_event, sdl3_poll_event_ll};
use glow::HasContext;
use sdl3::{
    Sdl, VideoSubsystem, event::{Event, WindowEvent}, video::{GLProfile, SwapInterval}
};
use vex_sdk_mock::{V5_DeviceT, V5_DeviceType};

fn create_glow_ctx(video: &VideoSubsystem) -> glow::Context {
    use std::ffi::c_void;

    unsafe { glow::Context::from_loader_function(|name| video.gl_get_proc_address(name).map(|f| f as *const c_void).unwrap_or(std::ptr::null())) }
}

struct WindowState {
    sdl: Sdl,
    video: VideoSubsystem,
    window: sdl3::video::Window,
    gl_context: sdl3::video::GLContext,
    gl: glow::Context,
    imgui_context: dear_imgui_rs::Context,
    last_poll: Instant,
    dt_poll: Duration,
    last_render: Instant,
    dt_render: Duration
}

struct ExtraArgs {
    voltages: [f64; 21],
    velocities: [f64; 21],
    positions: [f64; 21],
}

pub fn sim_main() {
    let mut window_state = init_window();

    let mut event_pump = window_state.sdl.event_pump().unwrap();

    let mut render_enabled = true;
    'main: loop {
        while let Some(event_raw) = sdl3_poll_event_ll() {
            if process_sys_event(&event_raw) {}
            let event = Event::from_ll(event_raw);
            match event {
                Event::Quit { .. } => { break 'main; }
                Event::KeyDown { keycode, .. } => {}
                Event::Window { window_id, win_event, .. } => {
                    match win_event {
                        WindowEvent::CloseRequested => {
                            if window_id == window_state.window.id() { break 'main; }
                        },
                        WindowEvent::Hidden => render_enabled = false,
                        WindowEvent::Exposed => render_enabled = true,
                        _ => {}
                    };
                },
                _ => {}
            };

            window_state.dt_poll = window_state.last_poll.elapsed();
            window_state.last_poll = Instant::now();
        }

        let mut voltages: [f64; 21] = [0.0; 21];
        let mut velocities: [f64; 21] = [0.0; 21];
        let mut positions: [f64; 21] = [0.0; 21];

        for i in 1_usize..21_usize {
            let device_ptr: V5_DeviceT = vex_sdk_mock::vexDeviceGetByIndex((i - 1) as u32);
            let mut device = unsafe { &mut *(*device_ptr as *mut vex_sdk_mock::SimDeviceWrapper) }.get();
            if device.device_type == V5_DeviceType::kDeviceTypeMotorSensor {
                break;
            }

            let voltage: f64 = unsafe { core::mem::transmute_copy(&device.data_in[2]) };
            let mut real_vel: f64 = unsafe { core::mem::transmute_copy(&device.data_out[1]) };
            let mut pos: f64 = unsafe { core::mem::transmute_copy(&device.data_out[1]) };

            pos += real_vel;
            real_vel += voltage / 12000.0 - 0.025;

            voltages[i-1] = voltage;
            velocities[i-1] = real_vel;
            positions[i-1] = pos;

            device.data_out[1] = unsafe { core::mem::transmute_copy(&real_vel) };
            device.data_out[2] = unsafe { core::mem::transmute_copy(&pos) };

            drop(device);
        }

        if render_enabled {
            render(&mut window_state, ExtraArgs { voltages, velocities, positions });
        }

        sleep(Duration::from_millis(33));
    }

    dear_imgui_sdl3::shutdown_for_opengl(&mut window_state.imgui_context);
    std::process::exit(0);
}

pub fn init_window() -> WindowState {
    let sdl = sdl3::init().unwrap();
    let video = sdl.video().unwrap();

    enable_native_ime_ui();

    {
        let gl_attr = video.gl_attr();
        gl_attr.set_context_profile(GLProfile::Core);
        gl_attr.set_context_version(3, 3);
    }

    let mut window = video.window("Dear ImGui + SDL3 OpenGL 3.3", 1280, 720).opengl().resizable().build().unwrap();
    let gl_context = window.gl_create_context().unwrap();
    window.gl_make_current(&gl_context).unwrap();

    video.gl_set_swap_interval(SwapInterval::VSync).ok();
    window.show();

    let gl = create_glow_ctx(&video);

    let mut imgui_context = dear_imgui_rs::Context::create();
    {
        let io = imgui_context.io_mut();
        let mut flags = io.config_flags();
        flags.insert(ConfigFlags::DOCKING_ENABLE);
        flags.insert(ConfigFlags::VIEWPORTS_ENABLE);
        io.set_config_flags(flags);
    }

    init_for_opengl(&mut imgui_context, &window, &gl_context, "#version 150").unwrap();

    WindowState { sdl, video, window, gl_context, gl, imgui_context, last_poll: Instant::now(), dt_poll: Duration::from_millis(0), last_render: Instant::now(), dt_render: Duration::from_millis(0) }
}

pub fn render(state: &mut WindowState, extras: ExtraArgs) {
    new_frame(&mut state.imgui_context);

    let ui = state.imgui_context.frame();
    ui.window("Hello").size([400.0, 300.0], dear_imgui_rs::Condition::FirstUseEver).build(|| {
        ui.text("Hello, World");
        for i in 0..20 {
            ui.text(format!("Motor {} - Position: {}; Velocity: {}; Voltage: {}", i+1, extras.positions[i], extras.velocities[i], extras.voltages[i]));
        }
    });

    let render_draw_data = state.imgui_context.render();
    unsafe {
        let (w, h) = state.window.size_in_pixels();
        state.gl.viewport(0, 0, w as i32, h as i32);
        state.gl.clear_color(0., 0., 0., 0.);
        state.gl.clear(glow::COLOR_BUFFER_BIT);
    }

    dear_imgui_sdl3::render(&render_draw_data);
    state.imgui_context.update_platform_windows();
    state.imgui_context.render_platform_windows_default();

    state.window.gl_make_current(&state.gl_context).ok();
    state.window.gl_swap_window();
    state.dt_render = state.last_render.elapsed();
    state.last_render = Instant::now();
}