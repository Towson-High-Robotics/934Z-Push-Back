//! Competition Control

#[unsafe(no_mangle)]
pub extern "C" fn vexCompetitionStatus() -> u32 {
    0b10
}
#[unsafe(no_mangle)]
pub extern "C" fn vexCompetitionControl(data: u32) {}
