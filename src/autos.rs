#[allow(dead_code)]
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Autos {
    RedLeft,
    RedRight,
    RedAWP,
    BlueLeft,
    BlueRight,
    BlueAWP,
    Skills,
    SkillsDriver,
    #[default]
    None,
}
