pub mod core;
pub mod tf;
pub mod ui;

fn main() {
    println!("Starting NavRS (Zero-Copy Rust Navigation Stack)...");
    ui::run_bevy_app();
}
