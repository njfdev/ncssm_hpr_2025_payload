import { useState, useEffect, memo } from "react";
import { listen } from "@tauri-apps/api/event";

const CameraView = memo(function CameraView() {
  const [frame, setFrame] = useState(null);
  const [frameCount, setFrameCount] = useState(0);

  useEffect(() => {
    const unlisten = listen("camera_frame", (event) => {
      setFrame(event.payload);
      setFrameCount((c) => c + 1);
    });
    return () => { unlisten.then((fn) => fn()); };
  }, []);

  if (!frame) {
    return (
      <section id="camera-view">
        <div className="panel camera-panel">
          <h3>Camera Feed</h3>
          <div className="camera-placeholder">
            Waiting for camera frames...
          </div>
        </div>
      </section>
    );
  }

  const sizeStr = frame.size_bytes < 1024
    ? frame.size_bytes + " B"
    : (frame.size_bytes / 1024).toFixed(1) + " KB";

  return (
    <section id="camera-view">
      <div className="panel camera-panel">
        <h3>Camera Feed</h3>
        <div className="camera-body">
          <img
            src={`data:${frame.mime_type};base64,${frame.image_base64}`}
            alt="Camera frame"
            className="camera-img"
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
      </div>
    </section>
  );
});

export default CameraView;
