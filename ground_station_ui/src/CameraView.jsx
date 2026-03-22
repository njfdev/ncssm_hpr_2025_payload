import { useState, useEffect, useCallback, memo } from "react";
import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";

const CameraView = memo(function CameraView({ connected, cameraCount, streamCamera }) {
  const [frame, setFrame] = useState(null);
  const [frameCount, setFrameCount] = useState(0);
  const [switching, setSwitching] = useState(false);

  useEffect(() => {
    const unlisten = listen("camera_frame", (event) => {
      setFrame(event.payload);
      setFrameCount((c) => c + 1);
      setSwitching(false);
    });
    return () => { unlisten.then((fn) => fn()); };
  }, []);

  const handleSwitch = useCallback(async (idx) => {
    if (idx === streamCamera || switching) return;
    setSwitching(true);
    try {
      await invoke("send_vehicle_command", { cmd: `switch_cam:${idx}` });
    } catch (e) {
      console.error("Switch camera failed:", e);
      setSwitching(false);
    }
  }, [streamCamera, switching]);

  const sizeStr = frame
    ? frame.size_bytes < 1024
      ? frame.size_bytes + " B"
      : (frame.size_bytes / 1024).toFixed(1) + " KB"
    : "";

  return (
    <section id="camera-view">
      <div className="panel camera-panel">
        <h3>
          Camera Feed
          {cameraCount > 1 && (
            <span className="camera-switcher">
              {Array.from({ length: cameraCount }, (_, i) => (
                <button
                  key={i}
                  className={`cam-btn${i === streamCamera ? " active" : ""}${switching ? " switching" : ""}`}
                  onClick={() => handleSwitch(i)}
                  disabled={!connected || (i === streamCamera && !switching)}
                  title={`Switch to camera ${i}`}
                >
                  CAM {i}
                </button>
              ))}
            </span>
          )}
        </h3>
        {!frame ? (
          <div className="camera-placeholder">
            {switching ? "Switching camera..." : "Waiting for camera frames..."}
          </div>
        ) : (
          <div className="camera-body">
            <img
              src={`data:${frame.mime_type};base64,${frame.image_base64}`}
              alt="Camera frame"
              className={`camera-img${switching ? " switching" : ""}`}
            />
            <div className="camera-stats">
              <span>{frame.width}x{frame.height}</span>
              <span>{frame.mime_type.split('/')[1].toUpperCase()}</span>
              <span>Q:{frame.quality}</span>
              <span>{sizeStr}</span>
              <span>{frame.fps.toFixed(1)} fps</span>
              <span>#{frameCount}</span>
            </div>
          </div>
        )}
      </div>
    </section>
  );
});

export default CameraView;
