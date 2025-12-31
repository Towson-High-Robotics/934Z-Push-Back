use std::{any::{Any, TypeId}, thread::sleep, time::Duration};

use dear_imgui_sdl3::{enable_native_ime_ui, init_for_opengl, init_platform_for_opengl, new_frame, process_sys_event, sdl3_new_frame, sdl3_poll_event_ll};
use sdl3::{sys::events::{SDL_EVENT_WINDOW_CLOSE_REQUESTED, SDL_WindowEvent}, video::GLProfile};

pub fn sim_main() {
    const ENABLE_VIEWPORTS: bool = true;

    let sdl = sdl3::init().unwrap();
    let video = sdl.video().unwrap();

    enable_native_ime_ui();

    {
        let gl_attr = video.gl_attr();
        gl_attr.set_context_profile(GLProfile::Core);
        gl_attr.set_context_version(3, 3);
    }

    let window = video
        .window("Dear ImGui + SDL3 OpenGL 3.3", 1280, 720)
        .opengl()
        .resizable()
        .build()
        .unwrap();
    let gl_context = window.gl_create_context().unwrap();
    window.gl_make_current(&gl_context).unwrap();

    let mut imgui_context = dear_imgui_rs::Context::create();

    init_for_opengl(&mut imgui_context, &window, &gl_context, "#version 150").unwrap();
    let mut event_pump = sdl.event_pump().unwrap();
    'main: loop {
        while let Some(event) = sdl3_poll_event_ll() {
            if process_sys_event(&event) {}
            match unsafe { event.r#type } {
                528_u32 /* SDL_EVENT_WINDOW_CLOSE_REQUESTED */ => {
                    break 'main;
                }
                _ => {}
            }
        }
        
        new_frame(&mut imgui_context);
        let ui = imgui_context.frame();
        ui.window("Hello")
            .size([400.0, 300.0], dear_imgui_rs::Condition::FirstUseEver)
            .build(|| {
                ui.text("Hello, World");
            });
        let render_draw_data = imgui_context.render();
        unsafe {
            use sdl3::video::Window;
            use sdl3::video::GLContext;
            window.gl_make_current(&gl_context).unwrap();
            
        }
        dear_imgui_sdl3::render(&render_draw_data);
        window.gl_swap_window();
        sleep(Duration::from_millis(33));
    }

    dear_imgui_sdl3::shutdown_for_opengl(&mut imgui_context);
}