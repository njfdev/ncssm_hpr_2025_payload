import { memo } from "react";

/** Coerce nullish values to 0 so .toFixed() never throws. */
const n = (v) => (v ?? 0);

function Field({ label, value, color }) {
  return (
    <div className="field">
      <span className="label">{label}</span>
      <span style={color ? { color } : undefined}>{value}</span>
    </div>
  );
}

function fixColor(fixType) {
  if (["3D Fix", "DGPS", "RTK Float", "RTK Fixed"].includes(fixType)) return "var(--green)";
  if (fixType === "2D Fix") return "var(--yellow)";
  return "var(--red)";
}

function hbDisplay(age) {
  if (age == null) return { text: "--", color: "var(--text-dim)" };
  return { text: n(age).toFixed(1) + " s ago", color: age < 3 ? "var(--green)" : "var(--red)" };
}

function bwDisplay(bps, pct) {
  bps = n(bps); pct = n(pct);
  let text = bps < 1024 ? bps.toFixed(0) + " B/s" : (bps / 1024).toFixed(1) + " KB/s";
  if (pct > 0) text += " (" + pct.toFixed(0) + "%)";
  const color = pct > 80 ? "var(--red)" : pct > 50 ? "var(--yellow)" : "var(--green)";
  return { text, color };
}

function lossColor(pct) {
  if (pct > 5) return "var(--red)";
  if (pct > 1) return "var(--yellow)";
  return "var(--green)";
}

const TelemetryPanels = memo(function TelemetryPanels({ snapshot: s, cursorActive, cursorTimeSec }) {
  const hb = hbDisplay(s.heartbeat_age_secs);
  const bw = bwDisplay(s.bytes_per_sec, s.bandwidth_pct);

  return (
    <section id="panels">
      {cursorActive && (
        <div className="cursor-banner">
          Viewing T+{n(cursorTimeSec).toFixed(1)}s &mdash; move cursor off chart for live data
        </div>
      )}

      {/* GPS */}
      <div className="panel">
        <h3>GPS</h3>
        <div className="panel-body">
          <Field label="Lat:" value={n(s.lat).toFixed(6)} />
          <Field label="Lon:" value={n(s.lon).toFixed(6)} />
          <Field label="Alt:" value={n(s.gps_alt).toFixed(2) + " m"} />
          <Field label="Fix:" value={s.fix_type || "No Fix"} color={fixColor(s.fix_type)} />
          <Field label="Sats:" value={n(s.satellites)} />
          <Field label="HDOP:" value={n(s.hdop).toFixed(2)} />
        </div>
      </div>

      {/* Barometer */}
      <div className="panel">
        <h3>Barometer</h3>
        <div className="panel-body">
          <Field label="Pressure:" value={n(s.pressure_hpa).toFixed(2) + " hPa"} />
          <Field label="Temp:" value={n(s.temperature_c).toFixed(2) + " \u00B0C"} />
          <Field label="Baro Alt:" value={n(s.baro_alt).toFixed(2) + " m"} />
          <Field label="Fused Alt:" value={n(s.fused_alt).toFixed(2) + " m"} />
          <Field label="Rel Alt:" value={n(s.fused_relative_alt).toFixed(2) + " m"} />
          <Field label="Vz:" value={n(s.vz).toFixed(2) + " m/s"} />
        </div>
      </div>

      {/* Accelerometer */}
      <div className="panel">
        <h3>Accelerometer</h3>
        <div className="panel-body">
          <Field label="X:" value={n(s.accel_x).toFixed(3) + " g"} color="var(--red)" />
          <Field label="Y:" value={n(s.accel_y).toFixed(3) + " g"} color="var(--green)" />
          <Field label="Z:" value={n(s.accel_z).toFixed(3) + " g"} color="var(--blue)" />
        </div>
      </div>

      {/* Gyroscope */}
      <div className="panel">
        <h3>Gyroscope</h3>
        <div className="panel-body">
          <Field label="X:" value={n(s.gyro_x).toFixed(3) + " \u00B0/s"} color="var(--red)" />
          <Field label="Y:" value={n(s.gyro_y).toFixed(3) + " \u00B0/s"} color="var(--green)" />
          <Field label="Z:" value={n(s.gyro_z).toFixed(3) + " \u00B0/s"} color="var(--blue)" />
        </div>
      </div>

      {/* Magnetometer */}
      <div className="panel">
        <h3>Magnetometer</h3>
        <div className="panel-body">
          <Field label="X:" value={n(s.mag_x).toFixed(2) + " \u00B5T"} color="var(--red)" />
          <Field label="Y:" value={n(s.mag_y).toFixed(2) + " \u00B5T"} color="var(--green)" />
          <Field label="Z:" value={n(s.mag_z).toFixed(2) + " \u00B5T"} color="var(--blue)" />
        </div>
      </div>

      {/* System */}
      <div className="panel">
        <h3>System</h3>
        <div className="panel-body">
          <Field label="Type:" value={s.mav_type || "Unknown"} />
          <Field label="Status:" value={s.system_status || "Unknown"} />
          <Field label="Heartbeat:" value={hb.text} color={hb.color} />
          <Field label="Msg Rate:" value={n(s.msg_rate).toFixed(1) + " msg/s"} />
          <Field label="Bandwidth:" value={bw.text} color={bw.color} />
          <Field label="Pkt Loss:" value={n(s.packet_loss_pct).toFixed(2) + "%"} color={lossColor(n(s.packet_loss_pct))} />
          <Field label="Total Msgs:" value={n(s.msg_count)} />
        </div>
      </div>
    </section>
  );
});

export default TelemetryPanels;
