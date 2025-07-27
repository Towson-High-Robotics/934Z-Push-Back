use core::{cell::RefCell, time::Duration};

use alloc::rc::Rc;
use vexide::time::sleep;

use crate::{comp_controller::{CompContState, CompController}, util::Robot};

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
        loop {
            let comp_cont = self.comp_cont.borrow();
            let comp_state = comp_cont.state;
            drop(comp_cont);
            if comp_state != CompContState::Disabled {
                self.render_comp_gui();
                sleep(Duration::from_secs_f64(1.0)).await;
            } else {
                self.render_gui();
                sleep(Duration::from_secs_f64(0.1)).await;
            }
        }
    }
}