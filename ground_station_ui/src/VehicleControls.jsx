import { useState } from "react";
import { invoke } from "@tauri-apps/api/core";

export default function VehicleControls({ connected }) {
  const [confirming, setConfirming] = useState(null);
  const [status, setStatus] = useState(null);
  const [recording, setRecording] = useState(true);

  const sendCommand = async (cmd, label) => {
    setStatus(null);
    try {
      await invoke("send_vehicle_command", { cmd });
      setStatus(`${label} sent`);
      if (cmd === "stop") setRecording(false);
      if (cmd === "start") setRecording(true);
      setTimeout(() => setStatus(null), 3000);
    } catch (e) {
      setStatus(`Error: ${e}`);
    }
    setConfirming(null);
  };

  const handleDangerous = (cmd, label) => {
    if (confirming === cmd) {
      sendCommand(cmd, label);
    } else {
      setConfirming(cmd);
      setTimeout(() => setConfirming((c) => (c === cmd ? null : c)), 3000);
    }
  };

  return (
    <div className="vehicle-controls">
      <span className="vehicle-controls-label">Vehicle:</span>
      <button
        className={`vehicle-btn stop-rec-btn${!recording ? " stopped" : ""}`}
        onClick={() => sendCommand(recording ? "stop" : "start", recording ? "Stop recording" : "Start recording")}
        disabled={!connected}
        title={recording ? "Stop video/audio recording (safe to power off)" : "Restart video/audio recording"}
      >
        {recording ? "Stop Recording" : "Start Recording"}
      </button>
      <button
        className="vehicle-btn calibrate-btn"
        onClick={() => sendCommand("calibrate_gyro", "Gyro calibrated")}
        disabled={!connected}
        title="Calibrate gyro bias (device must be stationary)"
      >
        Calibrate Gyro
      </button>
      <button
        className="vehicle-btn calibrate-btn"
        onClick={() => sendCommand("reset_orientation", "Orientation reset")}
        disabled={!connected}
        title="Zero orientation to current position"
      >
        Reset Orientation
      </button>
      <button
        className={`vehicle-btn shutdown-btn${confirming === "shutdown" ? " confirming" : ""}`}
        onClick={() => handleDangerous("shutdown", "Shutdown")}
        disabled={!connected}
        title="Shutdown the flight computer"
      >
        {confirming === "shutdown" ? "Confirm Shutdown?" : "Shutdown"}
      </button>
      <button
        className={`vehicle-btn reboot-btn${confirming === "reboot" ? " confirming" : ""}`}
        onClick={() => handleDangerous("reboot", "Reboot")}
        disabled={!connected}
        title="Reboot the flight computer"
      >
        {confirming === "reboot" ? "Confirm Reboot?" : "Reboot"}
      </button>
      {status && <span className="vehicle-status">{status}</span>}
    </div>
  );
}
