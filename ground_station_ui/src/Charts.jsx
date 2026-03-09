import { useRef, useEffect, useImperativeHandle, forwardRef, useCallback } from "react";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";

const n = (v) => (v ?? 0);
const MAX_DATA_POINTS = 60000; // ~50 min at 20Hz — memory cap
const PLOT_HEIGHT = 150;

function emptyData(numSeries) {
  const d = [[]];
  for (let i = 0; i < numSeries; i++) d.push([]);
  return d;
}

function pushPoint(data, t, values, maxPoints) {
  data[0].push(t);
  for (let i = 0; i < values.length; i++) data[i + 1].push(values[i]);
  while (data[0].length > maxPoints) {
    for (let i = 0; i < data.length; i++) data[i].shift();
  }
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

function makeOpts(width, seriesDefs, hooks) {
  return {
    width,
    height: PLOT_HEIGHT,
    cursor: {
      sync: { key: "telemetry" },
      drag: { x: true, y: false },
      points: { show: true, size: 6 },
    },
    select: { show: true },
    legend: { show: true, live: true },
    scales: { x: { time: false } },
    axes: [
      {
        stroke: "#666",
        grid: { stroke: "#333", width: 1 },
        ticks: { stroke: "#444" },
        font: "10px monospace",
        size: 30,
      },
      {
        stroke: "#666",
        grid: { stroke: "#333", width: 1 },
        ticks: { stroke: "#444" },
        font: "10px monospace",
        size: 55,
      },
    ],
    series: [{ label: "Time (s)" }, ...seriesDefs],
    hooks,
  };
}

const Charts = forwardRef(function Charts(
  { windowSec, isLive, sessionBand, onCursorMove, onPan, onGoLive, onTimeRange },
  ref
) {
  const containerRefs = useRef({});
  const chartsMap = useRef({});
  const dataArrays = useRef({});
  const onCursorMoveRef = useRef(onCursorMove);
  const onPanRef = useRef(onPan);
  const onGoLiveRef = useRef(onGoLive);
  const onTimeRangeRef = useRef(onTimeRange);
  const windowSecRef = useRef(windowSec);
  const isLiveRef = useRef(isLive);
  const sessionBandRef = useRef(sessionBand);
  const cursorRafRef = useRef(null);
  const loggingRef = useRef(false);

  useEffect(() => { onCursorMoveRef.current = onCursorMove; });
  useEffect(() => { onPanRef.current = onPan; });
  useEffect(() => { onGoLiveRef.current = onGoLive; });
  useEffect(() => { onTimeRangeRef.current = onTimeRange; });
  useEffect(() => { windowSecRef.current = windowSec; }, [windowSec]);
  useEffect(() => { isLiveRef.current = isLive; }, [isLive]);
  useEffect(() => { sessionBandRef.current = sessionBand; }, [sessionBand]);

  // Redraw charts when sessionBand changes (to update band highlighting)
  useEffect(() => {
    for (const def of CHART_DEFS) {
      const chart = chartsMap.current[def.id];
      const data = dataArrays.current[def.id];
      if (chart && data) chart.setData(data);
    }
  }, [sessionBand]);

  // When windowSec or isLive change externally, update chart scales
  useEffect(() => {
    if (!isLive) return;
    const allCharts = chartsMap.current;
    for (const def of CHART_DEFS) {
      const chart = allCharts[def.id];
      const data = dataArrays.current[def.id];
      if (!chart || !data || data[0].length === 0) continue;
      const latest = data[0][data[0].length - 1];
      if (windowSec === Infinity) {
        chart.setScale("x", { min: data[0][0], max: latest });
      } else {
        chart.setScale("x", { min: latest - windowSec, max: latest });
      }
    }
  }, [windowSec, isLive]);

  useEffect(() => {
    const allCharts = chartsMap.current;

    const reportTimeRange = () => {
      // Report from first chart's x-scale
      const firstChart = allCharts[CHART_DEFS[0]?.id];
      if (firstChart) {
        const scales = firstChart.scales.x;
        onTimeRangeRef.current?.({ min: scales.min, max: scales.max });
      }
    };

    const sharedHooks = {
      draw: [(u) => {
        const band = sessionBandRef.current;
        if (!band) return;
        const ctx = u.ctx;
        const xMin = u.valToPos(band.minSec, "x", true);
        const xMax = u.valToPos(band.maxSec, "x", true);
        const plotLeft = u.bbox.left / devicePixelRatio;
        const plotTop = u.bbox.top / devicePixelRatio;
        const plotWidth = u.bbox.width / devicePixelRatio;
        const plotHeight = u.bbox.height / devicePixelRatio;

        // Clamp to plot area
        const left = Math.max(xMin, plotLeft);
        const right = Math.min(xMax, plotLeft + plotWidth);
        if (right <= left) return;

        ctx.save();
        ctx.fillStyle = "rgba(0, 212, 255, 0.06)";
        ctx.fillRect(left, plotTop, right - left, plotHeight);

        // Draw vertical edge lines
        ctx.strokeStyle = "rgba(0, 212, 255, 0.3)";
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        if (xMin >= plotLeft && xMin <= plotLeft + plotWidth) {
          ctx.beginPath();
          ctx.moveTo(xMin, plotTop);
          ctx.lineTo(xMin, plotTop + plotHeight);
          ctx.stroke();
        }
        if (xMax >= plotLeft && xMax <= plotLeft + plotWidth) {
          ctx.beginPath();
          ctx.moveTo(xMax, plotTop);
          ctx.lineTo(xMax, plotTop + plotHeight);
          ctx.stroke();
        }
        ctx.restore();
      }],
      setCursor: [(u) => {
        const idx = u.cursor.idx;
        const time = (idx != null && u.data[0] && idx < u.data[0].length)
          ? u.data[0][idx]
          : null;
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
        onPanRef.current?.();
        reportTimeRange();
      }],
      setScale: [(_u, _key) => {
        reportTimeRange();
      }],
    };

    for (const def of CHART_DEFS) {
      const el = containerRefs.current[def.id];
      if (!el) continue;
      const w = el.clientWidth - 8;
      const data = emptyData(def.numSeries);
      dataArrays.current[def.id] = data;
      const chart = new uPlot(makeOpts(w, def.series, sharedHooks), data, el);
      allCharts[def.id] = chart;

      // Double-click → go live
      chart.over.addEventListener("dblclick", () => {
        onGoLiveRef.current?.();
      });

      // Mouse wheel → pan horizontally
      chart.over.addEventListener("wheel", (e) => {
        e.preventDefault();
        const wSec = windowSecRef.current;
        if (wSec === Infinity) return; // can't pan when showing all

        // Determine current visible range from this chart
        const xMin = chart.scales.x.min;
        const xMax = chart.scales.x.max;
        const range = xMax - xMin;
        const shift = range * 0.1 * Math.sign(e.deltaY || e.deltaX);

        // Clamp: don't pan before start of data
        const data = dataArrays.current[CHART_DEFS[0].id];
        if (!data || data[0].length === 0) return;
        const dataMin = data[0][0];
        const dataMax = data[0][data[0].length - 1];

        let newMin = xMin + shift;
        let newMax = xMax + shift;

        // Clamp to data bounds
        if (newMin < dataMin) {
          newMin = dataMin;
          newMax = dataMin + range;
        }
        if (newMax > dataMax) {
          newMax = dataMax;
          newMin = dataMax - range;
          if (newMin < dataMin) newMin = dataMin;
        }

        for (const c of Object.values(allCharts)) {
          c.setScale("x", { min: newMin, max: newMax });
        }

        onPanRef.current?.();
      }, { passive: false });
    }

    const handleResize = () => {
      for (const def of CHART_DEFS) {
        const el = containerRefs.current[def.id];
        const chart = allCharts[def.id];
        if (el && chart) {
          chart.setSize({ width: el.clientWidth - 8, height: PLOT_HEIGHT });
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
      const maxPts = loggingRef.current ? Infinity : MAX_DATA_POINTS;
      for (const def of CHART_DEFS) {
        const data = dataArrays.current[def.id];
        if (!data) continue;
        pushPoint(data, t, def.extract(snapshot), maxPts);
        chartsMap.current[def.id]?.setData(data);
      }
      // Auto-scroll when live
      if (isLiveRef.current) {
        const wSec = windowSecRef.current;
        for (const def of CHART_DEFS) {
          const chart = chartsMap.current[def.id];
          const data = dataArrays.current[def.id];
          if (!chart || !data || data[0].length === 0) continue;
          if (wSec === Infinity) {
            chart.setScale("x", { min: data[0][0], max: t });
          } else {
            chart.setScale("x", { min: t - wSec, max: t });
          }
        }
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
        // Set x-scale to show all loaded data
        if (data[0].length > 0) {
          chartsMap.current[def.id]?.setScale("x", {
            min: data[0][0],
            max: data[0][data[0].length - 1],
          });
        }
      }
    },
    setLogging(val) {
      loggingRef.current = val;
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
