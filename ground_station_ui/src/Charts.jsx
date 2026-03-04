import { useRef, useEffect, useImperativeHandle, forwardRef, useCallback } from "react";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";

const n = (v) => (v ?? 0);

function emptyData(numSeries) {
  const d = [[]];
  for (let i = 0; i < numSeries; i++) d.push([]);
  return d;
}

function pushPoint(data, t, values) {
  data[0].push(t);
  for (let i = 0; i < values.length; i++) data[i + 1].push(values[i]);
}

const CHART_DEFS = [
  {
    id: "altitude",
    title: "Altitude",
    series: [
      { label: "Baro", stroke: "#44ff44", width: 1.5 },
      { label: "Rel", stroke: "#4488ff", width: 1.5 },
    ],
    extract: (s) => [n(s.baro_alt), n(s.fused_relative_alt)],
    numSeries: 2,
  },
  {
    id: "accel",
    title: "Accelerometer",
    series: [
      { label: "X", stroke: "#ff4444", width: 1.5 },
      { label: "Y", stroke: "#44ff44", width: 1.5 },
      { label: "Z", stroke: "#4488ff", width: 1.5 },
    ],
    extract: (s) => [n(s.accel_x), n(s.accel_y), n(s.accel_z)],
    numSeries: 3,
  },
  {
    id: "gyro",
    title: "Gyroscope",
    series: [
      { label: "X", stroke: "#ff4444", width: 1.5 },
      { label: "Y", stroke: "#44ff44", width: 1.5 },
      { label: "Z", stroke: "#4488ff", width: 1.5 },
    ],
    extract: (s) => [n(s.gyro_x), n(s.gyro_y), n(s.gyro_z)],
    numSeries: 3,
  },
  {
    id: "pressure",
    title: "Pressure",
    series: [
      { label: "hPa", stroke: "#00ffcc", width: 1.5 },
    ],
    extract: (s) => [n(s.pressure_hpa)],
    numSeries: 1,
  },
];

function makeOpts(width, height, seriesDefs, hooks) {
  return {
    width,
    height,
    cursor: {
      sync: { key: "telemetry" },
      drag: { x: true, y: false },
    },
    select: { show: true },
    legend: { show: true, live: true },
    scales: { x: { time: false } },
    axes: [
      { stroke: "#666", grid: { stroke: "#333", width: 1 }, ticks: { stroke: "#444" }, font: "10px monospace" },
      { stroke: "#666", grid: { stroke: "#333", width: 1 }, ticks: { stroke: "#444" }, font: "10px monospace" },
    ],
    series: [{ label: "Time (s)" }, ...seriesDefs],
    hooks,
  };
}

const Charts = forwardRef(function Charts({ onCursorMove }, ref) {
  const containerRefs = useRef({});
  const chartsMap = useRef({});
  const dataArrays = useRef({});
  const onCursorMoveRef = useRef(onCursorMove);
  const cursorRafRef = useRef(null);

  // Keep callback ref in sync without triggering effect re-runs
  useEffect(() => { onCursorMoveRef.current = onCursorMove; });

  // Create all uPlot instances once on mount
  useEffect(() => {
    const allCharts = chartsMap.current;

    const sharedHooks = {
      setCursor: [(u) => {
        const idx = u.cursor.idx;
        const time = (idx != null && u.data[0] && idx < u.data[0].length)
          ? u.data[0][idx]
          : null;
        // Deduplicate via rAF — synced charts fire 4x per mouse-move
        if (cursorRafRef.current) cancelAnimationFrame(cursorRafRef.current);
        cursorRafRef.current = requestAnimationFrame(() => {
          onCursorMoveRef.current?.(time);
        });
      }],
      setSelect: [(u) => {
        if (u.select.width < 2) return;
        const min = u.posToVal(u.select.left, "x");
        const max = u.posToVal(u.select.left + u.select.width, "x");
        for (const chart of Object.values(allCharts)) {
          chart.setScale("x", { min, max });
        }
        u.setSelect({ left: 0, width: 0, top: 0, height: 0 }, false);
      }],
    };

    for (const def of CHART_DEFS) {
      const el = containerRefs.current[def.id];
      if (!el) continue;
      const w = el.clientWidth - 8;
      const h = el.clientHeight - 4;
      const data = emptyData(def.numSeries);
      dataArrays.current[def.id] = data;
      const chart = new uPlot(makeOpts(w, h, def.series, sharedHooks), data, el);
      allCharts[def.id] = chart;

      // Double-click resets zoom on all charts
      chart.over.addEventListener("dblclick", () => {
        for (const c of Object.values(allCharts)) {
          const d = c.data[0];
          if (d && d.length > 0) {
            c.setScale("x", { min: d[0], max: d[d.length - 1] });
          }
        }
      });
    }

    const handleResize = () => {
      for (const def of CHART_DEFS) {
        const el = containerRefs.current[def.id];
        const chart = allCharts[def.id];
        if (el && chart) {
          chart.setSize({ width: el.clientWidth - 8, height: el.clientHeight - 4 });
        }
      }
    };
    window.addEventListener("resize", handleResize);

    return () => {
      window.removeEventListener("resize", handleResize);
      if (cursorRafRef.current) cancelAnimationFrame(cursorRafRef.current);
      for (const chart of Object.values(allCharts)) chart.destroy();
    };
  }, []);

  useImperativeHandle(ref, () => ({
    push(snapshot) {
      const t = snapshot.timestamp_ms / 1000;
      for (const def of CHART_DEFS) {
        const data = dataArrays.current[def.id];
        if (!data) continue;
        pushPoint(data, t, def.extract(snapshot));
        chartsMap.current[def.id]?.setData(data);
      }
    },
    clear() {
      for (const def of CHART_DEFS) {
        const data = emptyData(def.numSeries);
        dataArrays.current[def.id] = data;
        chartsMap.current[def.id]?.setData(data);
      }
    },
    loadSession(snapshots) {
      for (const def of CHART_DEFS) {
        const data = emptyData(def.numSeries);
        for (const s of snapshots) {
          const t = s.timestamp_ms / 1000;
          data[0].push(t);
          const vals = def.extract(s);
          for (let i = 0; i < vals.length; i++) data[i + 1].push(vals[i]);
        }
        dataArrays.current[def.id] = data;
        chartsMap.current[def.id]?.setData(data);
      }
    },
  }), []);

  const setContainerRef = useCallback((id) => (el) => {
    containerRefs.current[id] = el;
  }, []);

  return (
    <section id="charts">
      {CHART_DEFS.map((def) => (
        <div className="chart-box" key={def.id}>
          <h3>{def.title}</h3>
          <div className="chart-container" ref={setContainerRef(def.id)} />
        </div>
      ))}
    </section>
  );
});

export default Charts;
