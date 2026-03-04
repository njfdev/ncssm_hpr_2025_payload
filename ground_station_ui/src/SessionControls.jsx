import { invoke } from "@tauri-apps/api/core";
import { save, open } from "@tauri-apps/plugin-dialog";

export default function SessionControls({ logging, onToggleLogging, onSessionLoaded }) {
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
        onSessionLoaded(snapshots);
      } catch (e) {
        console.error("Load failed:", e);
      }
    }
  };

  return (
    <div className="conn-right">
      <button className={logging ? "logging" : ""} onClick={onToggleLogging}>
        {logging ? "Stop Logging" : "Start Logging"}
      </button>
      <button onClick={handleSave} disabled={logging}>Save Session</button>
      <button onClick={handleLoad}>Load Session</button>
    </div>
  );
}
