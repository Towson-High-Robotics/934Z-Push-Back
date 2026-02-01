use crate::PROGRAM_START;

pub fn get_uptime_string() -> String {
    let uptime = PROGRAM_START.elapsed();
    format!("{:.0}:{:02}:{:0>6.3}", uptime.as_secs() / 3600, uptime.as_secs() / 60 % 60, uptime.as_secs_f64() % 60.0)
}

#[macro_export]
macro_rules! log_debug {
    () => {};
    ($($arg:tt)*) => {{
        use $crate::log::get_uptime_string;
        use colored::Colorize;
        let log_str = format!("{} [DEBUG]", get_uptime_string()).truecolor(0, 147, 255);
        eprintln!("{}", format!("{} {}", log_str, format!($($arg)*).truecolor(0, 147, 255)));
    }};
}

#[macro_export]
macro_rules! log_info {
    () => {};
    ($($arg:tt)*) => {{
        use $crate::log::get_uptime_string;
        use colored::Colorize;
        let log_str = format!("{} [INFO]", get_uptime_string()).truecolor(0, 147, 255);
        eprintln!("{}", format!("{} {}", log_str, format!($($arg)*)));
    }};
}

#[macro_export]
macro_rules! log_warn {
    () => {};
    ($($arg:tt)*) => {{
        use $crate::log::get_uptime_string;
        use colored::Colorize;
        let log_str = format!("{} [WARN]", get_uptime_string()).truecolor(255, 255, 0);
        eprintln!("{}", format!("{} {}", log_str, format!($($arg)*).truecolor(255, 255, 0)));
    }};
}

#[macro_export]
macro_rules! log_error {
    () => {};
    ($($arg:tt)*) => {{
        use $crate::log::get_uptime_string;
        use colored::Colorize;
        let log_str = format!("{} [ERROR]", get_uptime_string()).truecolor(255, 30, 0);
        eprintln!("{}", format!("{} {}", log_str, format!($($arg)*).truecolor(255, 30, 0)));
    }};
}

#[macro_export]
macro_rules! log_fatal {
    () => {};
    ($($arg:tt)*) => {{
        use $crate::log::get_uptime_string;
        use colored::Colorize;
        let log_str = format!("{} [FATAL]", get_uptime_string()).truecolor(255, 255, 255).on_truecolor(255, 30, 0);
        eprintln!("{}", format!("{} {}", log_str, format!($($arg)*).truecolor(255, 255, 255).on_truecolor(255, 30, 0)));
    }};
}
