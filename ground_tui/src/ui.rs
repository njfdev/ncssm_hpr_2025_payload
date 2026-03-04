use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Paragraph};
use ratatui::Frame;

use crate::app::TelemetryState;

pub fn draw(f: &mut Frame, state: &TelemetryState) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // title
            Constraint::Min(0),   // body
        ])
        .split(f.area());

    // Title bar
    let title = Paragraph::new(Line::from(vec![Span::styled(
        " NCSSM HPR Ground Station ",
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    )]))
    .block(Block::default().borders(Borders::ALL));
    f.render_widget(title, chunks[0]);

    // 3 rows x 2 columns
    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Ratio(1, 3),
            Constraint::Ratio(1, 3),
            Constraint::Ratio(1, 3),
        ])
        .split(chunks[1]);

    let row0 = split_horizontal(rows[0]);
    let row1 = split_horizontal(rows[1]);
    let row2 = split_horizontal(rows[2]);

    // Row 0: GPS | Barometer
    f.render_widget(gps_panel(state), row0[0]);
    f.render_widget(baro_panel(state), row0[1]);

    // Row 1: Accelerometer | Gyroscope
    f.render_widget(accel_panel(state), row1[0]);
    f.render_widget(gyro_panel(state), row1[1]);

    // Row 2: Magnetometer | System
    f.render_widget(mag_panel(state), row2[0]);
    f.render_widget(system_panel(state), row2[1]);
}

fn split_horizontal(area: Rect) -> Vec<Rect> {
    Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(area)
        .to_vec()
}

fn panel_block(title: &str) -> Block<'_> {
    Block::default()
        .title(format!(" {title} "))
        .borders(Borders::ALL)
        .border_style(Style::default().fg(Color::DarkGray))
        .title_style(Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD))
}

fn value_line<'a>(label: &'a str, value: String, color: Color) -> Line<'a> {
    Line::from(vec![
        Span::styled(
            format!("  {label:<12}"),
            Style::default().fg(Color::Gray),
        ),
        Span::styled(value, Style::default().fg(color)),
    ])
}

fn gps_panel(state: &TelemetryState) -> Paragraph<'_> {
    let fix_color = match state.fix_type.as_str() {
        "3D Fix" | "DGPS" | "RTK Fixed" | "RTK Float" => Color::Green,
        "2D Fix" => Color::Yellow,
        _ => Color::Red,
    };

    Paragraph::new(vec![
        value_line("Latitude", format!("{:.7}\u{00b0}", state.lat), Color::White),
        value_line("Longitude", format!("{:.7}\u{00b0}", state.lon), Color::White),
        value_line("Altitude", format!("{:.1} m", state.gps_alt), Color::Cyan),
        value_line("Fix", state.fix_type.clone(), fix_color),
        value_line("Satellites", format!("{}", state.satellites), Color::White),
        value_line("HDOP", format!("{:.2}", state.hdop), Color::White),
    ])
    .block(panel_block("GPS"))
}

fn baro_panel(state: &TelemetryState) -> Paragraph<'_> {
    Paragraph::new(vec![
        value_line("Pressure", format!("{:.2} hPa", state.pressure_hpa), Color::Cyan),
        value_line("Temperature", format!("{:.1} \u{00b0}C", state.temperature_c), Color::Yellow),
        value_line("Baro Alt", format!("{:.1} m", state.baro_alt), Color::Green),
        value_line("Fused Alt", format!("{:.1} m", state.fused_alt), Color::Green),
        value_line("Rel Alt", format!("{:.1} m", state.fused_relative_alt), Color::White),
        value_line("Vz", format!("{:.2} m/s", state.vz), Color::White),
    ])
    .block(panel_block("Barometer"))
}

fn xyz_lines(x: f32, y: f32, z: f32, unit: &str, color: Color) -> Vec<Line<'_>> {
    vec![
        value_line("X", format!("{x:>8.3} {unit}"), color),
        value_line("Y", format!("{y:>8.3} {unit}"), color),
        value_line("Z", format!("{z:>8.3} {unit}"), color),
    ]
}

fn accel_panel(state: &TelemetryState) -> Paragraph<'_> {
    Paragraph::new(xyz_lines(
        state.accel_x,
        state.accel_y,
        state.accel_z,
        "g",
        Color::Magenta,
    ))
    .block(panel_block("Accelerometer"))
}

fn gyro_panel(state: &TelemetryState) -> Paragraph<'_> {
    Paragraph::new(xyz_lines(
        state.gyro_x,
        state.gyro_y,
        state.gyro_z,
        "\u{00b0}/s",
        Color::Blue,
    ))
    .block(panel_block("Gyroscope"))
}

fn mag_panel(state: &TelemetryState) -> Paragraph<'_> {
    Paragraph::new(xyz_lines(
        state.mag_x,
        state.mag_y,
        state.mag_z,
        "\u{00b5}T",
        Color::LightCyan,
    ))
    .block(panel_block("Magnetometer"))
}

fn system_panel(state: &TelemetryState) -> Paragraph<'_> {
    let conn_color = if state.connected { Color::Green } else { Color::Red };
    let conn_text = if state.connected { "Connected" } else { "Disconnected" };

    let hb_text = match state.heartbeat_age_secs() {
        Some(age) if age < 3.0 => format!("{:.1}s ago", age),
        Some(age) => format!("{:.0}s ago (!)", age),
        None => "Never".into(),
    };
    let hb_color = match state.heartbeat_age_secs() {
        Some(age) if age < 3.0 => Color::Green,
        Some(_) => Color::Red,
        None => Color::DarkGray,
    };

    // Bandwidth display
    let bw_text = if state.max_bytes_per_sec > 0.0 {
        format!(
            "{:.0} / {:.0} B/s ({:.0}%)",
            state.bytes_per_sec,
            state.max_bytes_per_sec,
            state.bandwidth_pct()
        )
    } else {
        format!("{:.0} B/s", state.bytes_per_sec)
    };
    let bw_color = if state.bandwidth_pct() > 80.0 {
        Color::Red
    } else if state.bandwidth_pct() > 50.0 {
        Color::Yellow
    } else {
        Color::Green
    };

    // Packet loss display
    let loss_color = if state.packet_loss_pct > 5.0 {
        Color::Red
    } else if state.packet_loss_pct > 1.0 {
        Color::Yellow
    } else {
        Color::Green
    };
    let loss_text = format!(
        "{:.1}% ({} lost)",
        state.packet_loss_pct, state.total_lost
    );

    Paragraph::new(vec![
        value_line("Status", conn_text.into(), conn_color),
        value_line("Heartbeat", hb_text, hb_color),
        value_line("Msg Rate", format!("{:.1} msg/s", state.msg_rate), Color::Cyan),
        value_line("Bandwidth", bw_text, bw_color),
        value_line("Pkt Loss", loss_text, loss_color),
        value_line("Total Msgs", format!("{}", state.msg_count), Color::White),
    ])
    .block(panel_block("System"))
}
