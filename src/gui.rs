use core::{cell::RefCell, time::Duration};

use alloc::rc::Rc;
use vexide::{devices::{display::{Circle, Rect, RenderMode}, math::Point2}, prelude::{Display, Rgb}, time::sleep};

use crate::{comp_controller::{CompContState, CompController}, util::Robot};

fn draw_rounded_rect(display: &mut Display, start: (i16, i16), end: (i16, i16), rad: u8, color: Rgb<u8>) {
    let irad = i16::from(rad);
    let smr = (start.0 - irad, start.1 - irad);
    let epr = (end.0 + irad, end.1 + irad);
    display.fill(&Circle::new(Point2::from([smr.0, smr.1]), u16::from(rad)), color);
    display.fill(&Circle::new(Point2::from([epr.0, smr.1]), u16::from(rad)), color);
    display.fill(&Circle::new(Point2::from([epr.0, epr.1]), u16::from(rad)), color);
    display.fill(&Circle::new(Point2::from([smr.0, epr.1]), u16::from(rad)), color);
    display.fill(&Rect::new(Point2::from([smr.0, start.1]), Point2::from([epr.0, end.1])), color);
    display.fill(&Rect::new(Point2::from([start.0, smr.1]), Point2::from([end.0, epr.1])), color);
}

#[derive(Debug, Clone)]
pub(crate) struct Gui {
    robot: Rc<RefCell<Robot>>,
    comp_cont: Rc<RefCell<CompController>>
}

impl Gui {
    pub fn new(robot: Rc<RefCell<Robot>>, comp_cont: Rc<RefCell<CompController>>) -> Self {
        Self { robot, comp_cont }
    }

    fn render_comp_gui(&mut self) {

    }

    fn render_gui(&mut self) {

    }

    pub async fn render_loop(&mut self) {
        let mut robot = self.robot.borrow_mut();
        let mut disp = robot.take_display().unwrap();
        drop(robot);
        disp.set_render_mode(RenderMode::DoubleBuffered);
        let comp_gui_frametime = Duration::from_secs_f64(1.0);
        let gui_frametime = Duration::from_secs_f64(0.1);
        loop {
            let comp_cont = self.comp_cont.borrow();
            let comp_state = comp_cont.state;
            drop(comp_cont);
            if comp_state != CompContState::Disabled {
                self.render_comp_gui();
                sleep(comp_gui_frametime).await;
            } else {
                self.render_gui();
                sleep(gui_frametime).await;
            }
        }
    }
}