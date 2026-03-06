import { useState, useEffect, useCallback } from "react";
import { invoke } from "@tauri-apps/api/core";

export default function ConnectionBar({ connected, onConnect, onDisconnect }) {
  const [addr, setAddr] = useState("udpin:0.0.0.0:14550");
  const [ports, setPorts] = useState([]);
  const [error, setError] = useState(null);

  const refreshPorts = useCallback(async () => {
    const list = await invoke("list_serial_ports");
    setPorts(list);
  }, []);

  useEffect(() => { refreshPorts(); }, [refreshPorts]);

  const handlePortChange = (e) => {
    if (e.target.value) {
      setAddr("serial:" + e.target.value + ":115200");
    }
  };

  const handleConnect = async () => {
    setError(null);
    try {
      await onConnect(addr);
    } catch (e) {
      setError(String(e));
    }
  };

  const handleDisconnect = async () => {
    setError(null);
    try {
      await onDisconnect();
    } catch (e) {
      setError(String(e));
    }
  };

  return (
    <div className="conn-left">
      <select id="port-select" onChange={handlePortChange}>
        <option value="">-- Serial Ports --</option>
        {ports.map((p) => (
          <option key={p} value={p}>{p}</option>
        ))}
      </select>
      <button onClick={refreshPorts} title="Refresh ports">&#x21bb;</button>
      <input
        type="text"
        id="addr-input"
        value={addr}
        onChange={(e) => setAddr(e.target.value)}
        onKeyDown={(e) => { if (e.key === "Enter" && !connected) handleConnect(); }}
      />
      <button onClick={handleConnect} disabled={connected}>Connect</button>
      <button onClick={handleDisconnect} disabled={!connected}>Disconnect</button>
      <span
        id="status-indicator"
        className={connected ? "status-connected" : "status-disconnected"}
      >
        {error ? "Error: " + error : connected ? "Connected" : "Disconnected"}
      </span>
    </div>
  );
}
