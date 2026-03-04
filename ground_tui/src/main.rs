mod app;
mod mavlink_rx;
mod ui;

use std::io;
use std::sync::mpsc;
use std::time::Duration;

use clap::Parser;
use crossterm::event::{self, Event, KeyCode, KeyModifiers};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen};
use crossterm::ExecutableCommand;
use ratatui::backend::CrosstermBackend;
use ratatui::Terminal;

use app::TelemetryState;

#[derive(Parser)]
#[command(name = "ground_tui", about = "NCSSM HPR Ground Station TUI")]
struct Cli {
    /// MAVLink connection address (e.g. udpin:0.0.0.0:14550, serial:/dev/cu.usbmodem0021:57600)
    #[arg(long, default_value = "udpin:0.0.0.0:14550")]
    addr: String,
}

fn main() -> io::Result<()> {
    let cli = Cli::parse();

    // Set up panic hook to restore terminal
    let original_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let _ = disable_raw_mode();
        let _ = io::stdout().execute(LeaveAlternateScreen);
        original_hook(info);
    }));

    // MAVLink receiver thread
    let (tx, rx) = mpsc::channel();
    let _conn = mavlink_rx::spawn_receiver(&cli.addr, tx);

    // Terminal setup
    enable_raw_mode()?;
    io::stdout().execute(EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;

    let mut state = TelemetryState::new();

    // Main loop — 10 Hz redraw
    loop {
        // Drain all pending messages
        while let Ok(msg) = rx.try_recv() {
            state.update(&msg);
        }

        // Render
        terminal.draw(|f| ui::draw(f, &state))?;

        // Poll keyboard — 100ms timeout for ~10 Hz loop
        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                match key.code {
                    KeyCode::Char('q') => break,
                    KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => break,
                    KeyCode::Esc => break,
                    _ => {}
                }
            }
        }
    }

    // Restore terminal
    disable_raw_mode()?;
    io::stdout().execute(LeaveAlternateScreen)?;

    Ok(())
}
