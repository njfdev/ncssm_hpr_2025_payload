import { useState, useEffect, useRef, useCallback } from "react";
import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import ConnectionBar from "./ConnectionBar.jsx";
import TelemetryPanels from "./TelemetryPanels.jsx";
import ChartControls from "./ChartControls.jsx";
import Charts from "./Charts.jsx";
import SessionControls from "./SessionControls.jsx";
import CameraView from "./CameraView.jsx";
import VehicleControls from "./VehicleControls.jsx";

const MAX_ROLLING = 60000; // ~50 min at 20Hz

export const EMPTY_SNAPSHOT = {
  timestamp_ms: 0,
  lat: 0, lon: 0, gps_alt: 0, fix_type: "No Fix", satellites: 0, hdop: 0,
  fused_alt: 0, fused_relative_alt: 0, vz: 0,
  pressure_hpa: 0, temperature_c: 0, baro_alt: 0,
  accel_x: 0, accel_y: 0, accel_z: 0,
  gyro_x: 0, gyro_y: 0, gyro_z: 0,
  roll: 0, pitch: 0, yaw: 0,
  mag_x: 0, mag_y: 0, mag_z: 0,
  mav_type: "Unknown", system_status: "Unknown",
  connected: false, msg_count: 0, heartbeat_age_secs: null,
  msg_rate: 0, bytes_per_sec: 0, max_bytes_per_sec: 0,
  bandwidth_pct: 0, packet_loss_pct: 0,
  pico_connected: true,
  camera_recording: false,
  audio_recording: false,
  csv_logging: false,
  camera_count: 0,
  stream_camera: 0,
};

export default function App() {
  const [snapshot, setSnapshot] = useState(EMPTY_SNAPSHOT);
  const [cursorSnapshot, setCursorSnapshot] = useState(null);
  const [connected, setConnected] = useState(false);
  const [collecting, setCollecting] = useState(false);
  const [logging, setLogging] = useState(false);
  const [windowSec, setWindowSec] = useState(30);
  const [isLive, setIsLive] = useState(true);
  const [timeRange, setTimeRange] = useState(null);
  const [loadedFileName, setLoadedFileName] = useState(null);
  // Session band: {minSec, maxSec} — the time range of a recorded or loaded session
  const [sessionBand, setSessionBand] = useState(null);
  const chartsRef = useRef(null);
  const snapshotsRef = useRef([]);
  const collectingRef = useRef(false);
  const loggingRef = useRef(false);
  const logStartSecRef = useRef(null);

  useEffect(() => { collectingRef.current = collecting; }, [collecting]);
  useEffect(() => { loggingRef.current = logging; }, [logging]);

  // Listen for telemetry events
  useEffect(() => {
    const unlisten = listen("telemetry", (event) => {
      const s = { ...EMPTY_SNAPSHOT, ...event.payload };

      // Only feed data to charts/panels when collecting
      if (!collectingRef.current) return;

      setSnapshot(s);
      chartsRef.current?.push(s);

      snapshotsRef.current.push(s);
      if (!loggingRef.current && snapshotsRef.current.length > MAX_ROLLING) {
        snapshotsRef.current.shift();
      }
    });
    return () => { unlisten.then((fn) => fn()); };
  }, []);

  // Clear loaded session state and prepare for live data
  const clearLoadedSession = useCallback(() => {
    if (loadedFileName) {
      snapshotsRef.current = [];
      chartsRef.current?.clear();
      setLoadedFileName(null);
      setSessionBand(null);
      setWindowSec(30);
      setIsLive(true);
    }
  }, [loadedFileName]);

  const handleConnect = useCallback(async (addr) => {
    await invoke("connect", { addr });
    setConnected(true);
  }, []);

  const handleDisconnect = useCallback(async () => {
    await invoke("disconnect");
    setConnected(false);
    if (loggingRef.current) {
      try { await invoke("stop_logging"); } catch (_) {}
    }
    setCollecting(false);
    setLogging(false);
    setSnapshot(EMPTY_SNAPSHOT);
    setCursorSnapshot(null);
  }, []);

  const handleToggleCollecting = useCallback(() => {
    if (!collectingRef.current) {
      // Start collecting — clear loaded session if present
      clearLoadedSession();
      setCollecting(true);
    } else {
      // Pause — stop collecting and logging
      if (loggingRef.current) {
        invoke("stop_logging");
        chartsRef.current?.setLogging(false);
        setLogging(false);
        // Record session band end
        const snaps = snapshotsRef.current;
        if (snaps.length > 0 && logStartSecRef.current != null) {
          const endSec = snaps[snaps.length - 1].timestamp_ms / 1000;
          setSessionBand({ minSec: logStartSecRef.current, maxSec: endSec });
        }
        logStartSecRef.current = null;
      }
      setCollecting(false);
    }
  }, [clearLoadedSession]);

  const handleToggleLogging = useCallback(async () => {
    if (!loggingRef.current) {
      // Start logging: clear everything for a fresh session
      clearLoadedSession();
      snapshotsRef.current = [];
      chartsRef.current?.clear();
      chartsRef.current?.setLogging(true);
      setSessionBand(null);
      logStartSecRef.current = null; // will be set on first data point
      await invoke("start_logging");
      setLogging(true);
      setCollecting(true);
    } else {
      // Stop logging
      await invoke("stop_logging");
      chartsRef.current?.setLogging(false);
      setLogging(false);
      // Record session band
      const snaps = snapshotsRef.current;
      if (snaps.length > 0) {
        const startSec = snaps[0].timestamp_ms / 1000;
        const endSec = snaps[snaps.length - 1].timestamp_ms / 1000;
        setSessionBand({ minSec: startSec, maxSec: endSec });
      }
    }
  }, [clearLoadedSession]);

  const handleSessionLoaded = useCallback((snapshots, fileName) => {
    snapshotsRef.current = snapshots.map((s) => ({ ...EMPTY_SNAPSHOT, ...s }));
    chartsRef.current?.loadSession(snapshotsRef.current);
    if (snapshots.length > 0) {
      setSnapshot({ ...EMPTY_SNAPSHOT, ...snapshots[snapshots.length - 1] });
      const startSec = snapshots[0].timestamp_ms / 1000;
      const endSec = snapshots[snapshots.length - 1].timestamp_ms / 1000;
      setSessionBand({ minSec: startSec, maxSec: endSec });
    } else {
      setSessionBand(null);
    }
    setCursorSnapshot(null);
    setCollecting(false);
    setLoadedFileName(fileName);
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
          connected={connected}
          collecting={collecting}
          logging={logging}
          loadedFileName={loadedFileName}
          onToggleCollecting={handleToggleCollecting}
          onToggleLogging={handleToggleLogging}
          onSessionLoaded={handleSessionLoaded}
        />
      </header>
      <VehicleControls connected={connected} />

      <div className={`cursor-banner${cursorSnapshot != null ? " visible" : ""}`}>
        {cursorSnapshot != null
          ? `Viewing T+${(cursorSnapshot.timestamp_ms / 1000).toFixed(1)}s \u2014 move cursor off chart for live data`
          : "\u00A0"}
      </div>

      <main>
        <TelemetryPanels snapshot={displaySnapshot} />
        <CameraView
          connected={connected}
          cameraCount={displaySnapshot.camera_count}
          streamCamera={displaySnapshot.stream_camera}
        />
        <ChartControls
          windowSec={windowSec}
          isLive={isLive}
          timeRange={timeRange}
          loadedFileName={loadedFileName}
          logging={logging}
          onWindowChange={handleWindowChange}
          onGoLive={handleGoLive}
        />
        <Charts
          ref={chartsRef}
          windowSec={windowSec}
          isLive={isLive}
          sessionBand={sessionBand}
          onCursorMove={handleCursorMove}
          onPan={handlePan}
          onGoLive={handleGoLive}
          onTimeRange={handleTimeRange}
        />
      </main>
    </>
  );
}
