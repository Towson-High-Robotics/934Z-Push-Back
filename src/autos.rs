#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) enum SelectedAuto {
    RedLeft, RedRight, RedAWP,
    BlueLeft, BlueRight, BlueAWP,
    Skills, SkillsDriver, None
}

impl Default for SelectedAuto { fn default() -> Self { Self::None } }