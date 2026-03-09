import { invoke } from "@tauri-apps/api/core";
import { save, open } from "@tauri-apps/plugin-dialog";

export default function SessionControls({
  connected,
  collecting,
  logging,
  loadedFileName,
  onToggleCollecting,
  onToggleLogging,
  onSessionLoaded,
}) {
  const handleSave = async () => {
    const path = await save({
      filters: [{ name: "JSON", extensions: ["json"] }],
      defaultPath: "session.json",
    });
    if (path) {
      try {
        await invoke("save_session", { path });
      } catch (e) {
        console.error("Save failed:", e);
      }
    }
  };

  const handleLoad = async () => {
    const path = await open({
      filters: [{ name: "JSON", extensions: ["json"] }],
      multiple: false,
    });
    if (path) {
      try {
        const snapshots = await invoke("load_session", { path });
        const fileName = path.split(/[/\\]/).pop() || path;
        onSessionLoaded(snapshots, fileName);
      } catch (e) {
        console.error("Load failed:", e);
      }
    }
  };

  return (
    <div className="conn-right">
      <button
        className={`collect-btn${collecting ? " active" : ""}`}
        onClick={onToggleCollecting}
        disabled={!connected}
        title={collecting ? "Pause data collection" : "Start displaying live data"}
      >
        {collecting ? "\u23F8" : "\u25B6"}
      </button>
      <button
        className={logging ? "logging" : ""}
        onClick={onToggleLogging}
        disabled={!connected}
        title={logging ? "Stop recording session" : "Start recording session"}
      >
        {logging ? "\u23F9 Stop Logging" : "\u23FA Start Logging"}
      </button>
      <button onClick={handleSave} disabled={logging} title="Save recorded session to file">
        Save Session
      </button>
      <button onClick={handleLoad} title="Load session from file">
        Load Session
      </button>
    </div>
  );
}
