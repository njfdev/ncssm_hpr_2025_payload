import { memo } from "react";

const WINDOW_OPTIONS = [
  { label: "10s", value: 10 },
  { label: "30s", value: 30 },
  { label: "1m", value: 60 },
  { label: "2m", value: 120 },
  { label: "5m", value: 300 },
  { label: "All", value: Infinity },
];

function formatTime(sec) {
  if (sec == null || !isFinite(sec)) return "--";
  return "T+" + sec.toFixed(1) + "s";
}

const ChartControls = memo(function ChartControls({
  windowSec,
  isLive,
  timeRange,
  onWindowChange,
  onGoLive,
}) {
  return (
    <div className="chart-controls">
      <div className="chart-controls-group">
        <span className="chart-controls-label">Window:</span>
        {WINDOW_OPTIONS.map((opt) => (
          <button
            key={opt.label}
            className={`window-btn${windowSec === opt.value ? " active" : ""}`}
            onClick={() => onWindowChange(opt.value)}
          >
            {opt.label}
          </button>
        ))}
      </div>
      <div className="chart-controls-group">
        <button
          className={`live-btn${isLive ? " active" : ""}`}
          onClick={onGoLive}
        >
          LIVE
        </button>
      </div>
      <div className="chart-controls-group time-range">
        {timeRange
          ? `${formatTime(timeRange.min)} \u2014 ${formatTime(timeRange.max)}`
          : "\u00A0"}
      </div>
    </div>
  );
});

export default ChartControls;
