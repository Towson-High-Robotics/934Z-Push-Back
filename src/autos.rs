#[allow(dead_code)]
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Autos {
    RedLeft, RedRight, RedAWP,
    BlueLeft, BlueRight, BlueAWP,
    Skills, SkillsDriver, None
}

impl Default for Autos { fn default() -> Self { Self::None } }