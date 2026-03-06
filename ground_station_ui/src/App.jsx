import { useState, useEffect, useRef, useCallback } from "react";
import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import ConnectionBar from "./ConnectionBar.jsx";
import TelemetryPanels from "./TelemetryPanels.jsx";
import ChartControls from "./ChartControls.jsx";
import Charts from "./Charts.jsx";
import SessionControls from "./SessionControls.jsx";

const MAX_ROLLING = 60000; // ~50 min at 20Hz

export const EMPTY_SNAPSHOT = {
  timestamp_ms: 0,
  lat: 0, lon: 0, gps_alt: 0, fix_type: "No Fix", satellites: 0, hdop: 0,
  fused_alt: 0, fused_relative_alt: 0, vz: 0,
  pressure_hpa: 0, temperature_c: 0, baro_alt: 0,
  accel_x: 0, accel_y: 0, accel_z: 0,
  gyro_x: 0, gyro_y: 0, gyro_z: 0,
  mag_x: 0, mag_y: 0, mag_z: 0,
  mav_type: "Unknown", system_status: "Unknown",
  connected: false, msg_count: 0, heartbeat_age_secs: null,
  msg_rate: 0, bytes_per_sec: 0, max_bytes_per_sec: 0,
  bandwidth_pct: 0, packet_loss_pct: 0,
};

export default function App() {
  const [snapshot, setSnapshot] = useState(EMPTY_SNAPSHOT);
  const [cursorSnapshot, setCursorSnapshot] = useState(null);
  const [connected, setConnected] = useState(false);
  const [logging, setLogging] = useState(false);
  const [windowSec, setWindowSec] = useState(30);
  const [isLive, setIsLive] = useState(true);
  const [timeRange, setTimeRange] = useState(null);
  const chartsRef = useRef(null);
  const snapshotsRef = useRef([]);
  const loggingRef = useRef(false);

  useEffect(() => { loggingRef.current = logging; }, [logging]);

  // Listen for telemetry events
  useEffect(() => {
    const unlisten = listen("telemetry", (event) => {
      const s = { ...EMPTY_SNAPSHOT, ...event.payload };
      setSnapshot(s);

      chartsRef.current?.push(s);

      snapshotsRef.current.push(s);
      if (!loggingRef.current && snapshotsRef.current.length > MAX_ROLLING) {
        snapshotsRef.current.shift();
      }
    });
    return () => { unlisten.then((fn) => fn()); };
  }, []);

  const handleConnect = useCallback(async (addr) => {
    await invoke("connect", { addr });
    setConnected(true);
  }, []);

  const handleDisconnect = useCallback(async () => {
    await invoke("disconnect");
    setConnected(false);
    setSnapshot(EMPTY_SNAPSHOT);
    setCursorSnapshot(null);
  }, []);

  const handleToggleLogging = useCallback(async () => {
    if (!loggingRef.current) {
      snapshotsRef.current = [];
      chartsRef.current?.clear();
      chartsRef.current?.setLogging(true);
      await invoke("start_logging");
      setLogging(true);
    } else {
      await invoke("stop_logging");
      chartsRef.current?.setLogging(false);
      setLogging(false);
    }
  }, []);

  const handleSessionLoaded = useCallback((snapshots) => {
    snapshotsRef.current = snapshots.map((s) => ({ ...EMPTY_SNAPSHOT, ...s }));
    chartsRef.current?.loadSession(snapshotsRef.current);
    if (snapshots.length > 0) {
      setSnapshot({ ...EMPTY_SNAPSHOT, ...snapshots[snapshots.length - 1] });
    }
    // Show all data when loading a session
    setWindowSec(Infinity);
    setIsLive(false);
  }, []);

  const handleCursorMove = useCallback((timeSec) => {
    if (timeSec == null) {
      setCursorSnapshot(null);
      return;
    }
    const snaps = snapshotsRef.current;
    if (snaps.length === 0) { setCursorSnapshot(null); return; }

    const targetMs = timeSec * 1000;
    let lo = 0, hi = snaps.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (snaps[mid].timestamp_ms < targetMs) lo = mid + 1;
      else hi = mid;
    }
    let best = lo;
    if (lo > 0) {
      const dLo = Math.abs(snaps[lo].timestamp_ms - targetMs);
      const dPrev = Math.abs(snaps[lo - 1].timestamp_ms - targetMs);
      if (dPrev < dLo) best = lo - 1;
    }
    setCursorSnapshot(snaps[best]);
  }, []);

  const handlePan = useCallback(() => {
    setIsLive(false);
  }, []);

  const handleGoLive = useCallback(() => {
    setIsLive(true);
  }, []);

  const handleWindowChange = useCallback((sec) => {
    setWindowSec(sec);
    setIsLive(true);
  }, []);

  const handleTimeRange = useCallback((range) => {
    setTimeRange(range);
  }, []);

  const displaySnapshot = cursorSnapshot ?? snapshot;

  return (
    <>
      <header id="connection-bar">
        <ConnectionBar
          connected={connected}
          onConnect={handleConnect}
          onDisconnect={handleDisconnect}
        />
        <SessionControls
          logging={logging}
          onToggleLogging={handleToggleLogging}
          onSessionLoaded={handleSessionLoaded}
        />
      </header>

      <div className={`cursor-banner${cursorSnapshot != null ? " visible" : ""}`}>
        {cursorSnapshot != null
          ? `Viewing T+${(cursorSnapshot.timestamp_ms / 1000).toFixed(1)}s \u2014 move cursor off chart for live data`
          : "\u00A0"}
      </div>

      <main>
        <TelemetryPanels snapshot={displaySnapshot} />
        <ChartControls
          windowSec={windowSec}
          isLive={isLive}
          timeRange={timeRange}
          onWindowChange={handleWindowChange}
          onGoLive={handleGoLive}
        />
        <Charts
          ref={chartsRef}
          windowSec={windowSec}
          isLive={isLive}
          onCursorMove={handleCursorMove}
          onPan={handlePan}
          onGoLive={handleGoLive}
          onTimeRange={handleTimeRange}
        />
      </main>
    </>
  );
}
