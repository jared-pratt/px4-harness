/**
 * UAV Simulation Dashboard — client JS
 *
 * Sections:
 *   1. WebSocket
 *   2. Drone card telemetry rendering
 *   3. Drone actions (REST)
 *   4. Leaflet map (flight path)
 *   5. Chart.js real-time graphs
 *   6. Flight log analysis (past runs)
 *   7. Scenario editor
 *   8. Failure injection
 *   9. Mission control
 *  10. Console log
 *  11. Init
 */

"use strict";

// ═══════════════════════════════════════════════════════════
// 1. WebSocket
// ═══════════════════════════════════════════════════════════

let _ws = null;
let _wsReconnectTimer = null;

function wsConnect() {
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  _ws = new WebSocket(`${protocol}://${location.host}/ws`);
  setBadge("connecting");

  _ws.onopen = () => {
    setBadge("connected");
    consoleLog("WebSocket connected", "action");
    clearInterval(_ws._ping);
    _ws._ping = setInterval(() => { if (_ws.readyState === 1) _ws.send("ping"); }, 15000);
    // Fetch history for map trails on reconnect
    fetchHistory();
  };

  _ws.onmessage = (evt) => {
    try { handleUpdate(JSON.parse(evt.data)); }
    catch (e) { console.error("WS parse:", e); }
  };

  _ws.onerror = () => setBadge("disconnected");

  _ws.onclose = () => {
    setBadge("disconnected");
    clearInterval(_ws._ping);
    consoleLog("WebSocket disconnected — reconnecting in 3 s…", "warn");
    clearTimeout(_wsReconnectTimer);
    _wsReconnectTimer = setTimeout(wsConnect, 3000);
  };
}

function setBadge(state) {
  const el = document.getElementById("ws-badge");
  el.className = `ws-badge ${state}`;
  el.textContent = { connected: "● Connected", disconnected: "○ Disconnected", connecting: "◌ Connecting…" }[state] || state;
}

function handleUpdate(data) {
  if (data.type !== "update") return;
  if (data.drones) {
    for (const [id, tel] of Object.entries(data.drones)) {
      const did = parseInt(id);
      updateDroneCard(did, tel);
      if (tel.connected) {
        addGraphPoint(did, tel);
        updateMapMarker(did, tel);
      }
    }
  }
  if (data.logs) syncServerLogs(data.logs);
}

// ═══════════════════════════════════════════════════════════
// 2. Drone card rendering
// ═══════════════════════════════════════════════════════════

function updateDroneCard(id, tel) {
  const card = document.getElementById(`drone-${id}`);
  if (!card) return;
  const connected = tel.connected === true;
  card.classList.toggle("offline", !connected);

  const cb = card.querySelector(".conn-badge");
  cb.textContent = connected ? "ONLINE" : "OFFLINE";
  cb.className = `conn-badge ${connected ? "online" : "offline"}`;

  if (!connected) return;

  const modeEl = card.querySelector(".d-mode");
  if (modeEl) { modeEl.textContent = tel.flight_mode || "—"; modeEl.className = "tel-value " + modeColor(tel.flight_mode); }

  setText(card, ".d-armed",  tel.armed  ? "ARMED"  : "DISARMED", tel.armed  ? "red" : "muted");
  setText(card, ".d-inair",  tel.in_air ? "YES"    : "NO",       tel.in_air ? "green" : "muted");
  setText(card, ".d-alt",    tel.rel_alt_m != null ? `${tel.rel_alt_m.toFixed(1)} m` : "—");
  setText(card, ".d-speed",  tel.speed_m_s != null ? `${tel.speed_m_s.toFixed(1)} m/s` : "—");

  const pct = tel.battery_pct ?? 0;
  setText(card, ".d-batt", `${pct.toFixed(0)}%`);
  const fill = card.querySelector(".batt-fill");
  if (fill) {
    fill.style.width = `${Math.min(100, Math.max(0, pct))}%`;
    fill.className = "batt-fill" + (pct < 10 ? " danger" : pct < 25 ? " warn" : "");
  }

  const posEl = card.querySelector(".d-pos");
  if (posEl && tel.lat != null) {
    posEl.textContent = `${tel.lat.toFixed(6)}, ${tel.lon.toFixed(6)}  (abs ${tel.abs_alt_m} m)`;
  }

  const healthEl = card.querySelector(".d-health");
  if (healthEl) {
    healthEl.innerHTML = [["Gyro",tel.gyro_ok],["Accel",tel.accel_ok],["Mag",tel.mag_ok],["GPS",tel.gps_ok],["Home",tel.home_ok]]
      .map(([n,ok]) => `<span class="health-chip ${ok===true?"ok":ok===false?"fail":"unk"}">${n} ${ok===true?"✓":ok===false?"✗":"?"}</span>`).join("");
  }
}

function modeColor(mode) {
  if (!mode) return "muted";
  const m = mode.toUpperCase();
  if (["LAND","LANDING"].includes(m))          return "yellow";
  if (["RTL","RETURN_TO_LAUNCH"].includes(m))  return "yellow";
  if (["HOLD","LOITER","POSCTL"].includes(m))  return "green";
  if (["MISSION","AUTO_MISSION"].includes(m))  return "green";
  if (["UNKNOWN","—"].includes(m))             return "muted";
  return "";
}

function setText(parent, sel, text, colorClass = "") {
  const el = parent.querySelector(sel);
  if (!el) return;
  el.textContent = text;
  el.className = "tel-value " + colorClass;
}

// ═══════════════════════════════════════════════════════════
// 3. Drone actions
// ═══════════════════════════════════════════════════════════

async function droneAction(droneId, action, params = {}) {
  consoleLog(`→ drone${droneId}: ${action} ${JSON.stringify(params)}`, "action");
  try {
    const r = await fetch(`/api/drone/${droneId}/action`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action, params }),
    });
    if (!r.ok) { const d = await r.json(); consoleLog(`✗ ${d.detail || r.statusText}`, "error"); }
  } catch (e) { consoleLog(`✗ drone${droneId}: ${e}`, "error"); }
}

async function droneActionTakeoff(droneId) {
  const altEl = document.getElementById(`d${droneId}-takeoff-alt`);
  const alt = parseFloat(altEl?.value || 5) || 5;
  await droneAction(droneId, "takeoff", { alt });
}

async function droneGotoOffset(droneId) {
  const north = parseFloat(document.getElementById(`d${droneId}-north`)?.value || 0);
  const east  = parseFloat(document.getElementById(`d${droneId}-east`)?.value  || 0);
  const alt   = parseFloat(document.getElementById(`d${droneId}-alt`)?.value   || 10);
  await droneAction(droneId, "goto_offset", { north_m: north, east_m: east, rel_alt_m: alt });
}

// ═══════════════════════════════════════════════════════════
// 4. Leaflet map
// ═══════════════════════════════════════════════════════════

let _map = null;
const _mapMarkers = {};
const _mapTrails  = {};
const _trailPoints = { 1: [], 2: [] };
let _mapInitialized = false;

const DRONE_COLORS = { 1: "#58a6ff", 2: "#3fb950" };

function initMap() {
  _map = L.map("map", { preferCanvas: true }).setView([0, 0], 2);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    attribution: "© OpenStreetMap contributors",
    maxZoom: 19,
  }).addTo(_map);

  [1, 2].forEach(id => {
    const color = DRONE_COLORS[id];

    // Custom SVG marker icon
    const icon = L.divIcon({
      html: `<svg width="20" height="20" viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">
               <circle cx="10" cy="10" r="8" fill="${color}" fill-opacity="0.9" stroke="#0d1117" stroke-width="2"/>
               <text x="10" y="14" font-size="10" text-anchor="middle" fill="#0d1117" font-weight="bold">${id}</text>
             </svg>`,
      className: "",
      iconSize: [20, 20],
      iconAnchor: [10, 10],
    });

    _mapMarkers[id] = L.marker([0, 0], { icon })
      .addTo(_map)
      .bindPopup(`<b>Drone ${id}</b><br><span id="map-popup-${id}">No data</span>`);

    _mapTrails[id] = L.polyline([], {
      color,
      weight: 2,
      opacity: 0.8,
      dashArray: "4 4",
    }).addTo(_map);
  });
}

function updateMapMarker(droneId, tel) {
  if (!_map || !tel.lat || !tel.lon) return;
  const lat = tel.lat, lon = tel.lon;

  // First fix: center map
  if (!_mapInitialized && (lat !== 0 || lon !== 0)) {
    _map.setView([lat, lon], 17);
    _mapInitialized = true;
  }

  _mapMarkers[droneId].setLatLng([lat, lon]);
  const popup = document.getElementById(`map-popup-${droneId}`);
  if (popup) popup.innerHTML =
    `Mode: <b>${tel.flight_mode}</b> | Alt: <b>${tel.rel_alt_m}m</b> | Batt: <b>${tel.battery_pct}%</b>`;

  // Append to trail (throttle: only add a point if moved >0.5 m or first point)
  const pts = _trailPoints[droneId];
  if (pts.length === 0 || haversineM(pts[pts.length - 1], [lat, lon]) > 0.5) {
    pts.push([lat, lon]);
    if (pts.length > 500) pts.shift();
    _mapTrails[droneId].setLatLngs(pts);
  }
}

async function fetchHistory() {
  for (const id of [1, 2]) {
    try {
      const r = await fetch(`/api/history/${id}`);
      const d = await r.json();
      if (!d.history?.length) continue;
      _trailPoints[id] = d.history.map(h => [h.lat, h.lon]);
      _mapTrails[id].setLatLngs(_trailPoints[id]);
      const last = d.history[d.history.length - 1];
      _mapMarkers[id].setLatLng([last.lat, last.lon]);
      if (!_mapInitialized && (last.lat !== 0 || last.lon !== 0)) {
        _map.setView([last.lat, last.lon], 17);
        _mapInitialized = true;
      }
    } catch (_) {}
  }
}

function fitMapToDrones() {
  const pts = [..._trailPoints[1], ..._trailPoints[2]].filter(p => p[0] !== 0 || p[1] !== 0);
  if (pts.length > 0) _map.fitBounds(L.latLngBounds(pts), { padding: [40, 40] });
}

function clearTrails() {
  [1, 2].forEach(id => {
    _trailPoints[id] = [];
    _mapTrails[id].setLatLngs([]);
  });
}

function haversineM([lat1, lon1], [lat2, lon2]) {
  const R = 6371000;
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const a = Math.sin(dLat/2)**2 + Math.cos(lat1*Math.PI/180) * Math.cos(lat2*Math.PI/180) * Math.sin(dLon/2)**2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
}

// ═══════════════════════════════════════════════════════════
// 5. Chart.js real-time graphs
// ═══════════════════════════════════════════════════════════

const GRAPH_BUF = 120;   // ~30 s at 4 Hz
const _gBuf = {
  1: { alt: [], speed: [], batt: [], t: [] },
  2: { alt: [], speed: [], batt: [], t: [] },
};

let _charts = {};
let _chartUpdateTimer = null;

function initCharts() {
  const opts = (yLabel, yUnit, color1, color2) => ({
    animation: false,
    responsive: true,
    maintainAspectRatio: true,
    interaction: { mode: "index", intersect: false },
    plugins: {
      legend: { labels: { color: "#8b949e", font: { size: 11 } } },
      tooltip: { backgroundColor: "#161b22", borderColor: "#30363d", borderWidth: 1,
                 titleColor: "#e6edf3", bodyColor: "#8b949e" },
    },
    scales: {
      x: { display: false, ticks: { color: "#8b949e" } },
      y: { title: { display: true, text: yUnit, color: "#8b949e", font: { size: 11 } },
           ticks: { color: "#8b949e", font: { size: 11 } },
           grid:  { color: "#21262d" } },
    },
  });

  const mkDatasets = (label1, label2, c1, c2) => [
    { label: label1, data: [], borderColor: c1, backgroundColor: c1 + "22",
      borderWidth: 1.5, pointRadius: 0, tension: 0.3, fill: true },
    { label: label2, data: [], borderColor: c2, backgroundColor: c2 + "22",
      borderWidth: 1.5, pointRadius: 0, tension: 0.3, fill: true },
  ];

  _charts.alt   = new Chart(document.getElementById("chart-alt"), {
    type: "line",
    data: { labels: [], datasets: mkDatasets("Drone 1 alt (m)", "Drone 2 alt (m)", "#58a6ff", "#3fb950") },
    options: opts("Altitude", "m", "#58a6ff", "#3fb950"),
  });

  _charts.speed = new Chart(document.getElementById("chart-speed"), {
    type: "line",
    data: { labels: [], datasets: mkDatasets("Drone 1 speed", "Drone 2 speed", "#58a6ff", "#3fb950") },
    options: opts("Speed", "m/s", "#58a6ff", "#3fb950"),
  });

  _charts.batt  = new Chart(document.getElementById("chart-batt"), {
    type: "line",
    data: { labels: [], datasets: mkDatasets("Drone 1 batt %", "Drone 2 batt %", "#58a6ff", "#3fb950") },
    options: opts("Battery", "%", "#58a6ff", "#3fb950"),
  });

  // Refresh charts at 2 Hz (not every WebSocket message)
  _chartUpdateTimer = setInterval(flushCharts, 500);
}

function addGraphPoint(droneId, tel) {
  const b = _gBuf[droneId];
  const ts = new Date().toLocaleTimeString("en-US", { hour12: false,
    hour: "2-digit", minute: "2-digit", second: "2-digit" });
  b.t.push(ts);
  b.alt.push(tel.rel_alt_m ?? null);
  b.speed.push(tel.speed_m_s ?? null);
  b.batt.push(tel.battery_pct ?? null);
  if (b.t.length > GRAPH_BUF) { b.t.shift(); b.alt.shift(); b.speed.shift(); b.batt.shift(); }
}

function flushCharts() {
  if (!_charts.alt) return;

  // Build unified time labels (union of both drones, trimmed to GRAPH_BUF)
  const labels = _gBuf[1].t.length >= _gBuf[2].t.length ? _gBuf[1].t : _gBuf[2].t;

  const update = (chart, key) => {
    chart.data.labels = labels;
    chart.data.datasets[0].data = _gBuf[1][key];
    chart.data.datasets[1].data = _gBuf[2][key];
    chart.update("none");
  };
  update(_charts.alt,   "alt");
  update(_charts.speed, "speed");
  update(_charts.batt,  "batt");
}

function showGraph(name, btn) {
  ["alt", "speed", "batt"].forEach(n => {
    document.getElementById(`chart-${n}`).style.display = n === name ? "" : "none";
  });
  document.querySelectorAll(".tab-btn").forEach(b => b.classList.remove("active"));
  if (btn) btn.classList.add("active");
}

// ═══════════════════════════════════════════════════════════
// 6. Flight log analysis
// ═══════════════════════════════════════════════════════════

let _logCharts = {};

async function loadRuns() {
  try {
    const r = await fetch("/api/runs");
    const d = await r.json();
    const sel = document.getElementById("run-select");
    if (!d.runs.length) {
      sel.innerHTML = `<option disabled>No runs found — run a scenario first</option>`;
      return;
    }
    sel.innerHTML = d.runs.map(run =>
      `<option value="${run.name}">${run.name} (${run.rows} rows)</option>`
    ).join("");
  } catch (e) { consoleLog("Could not load runs: " + e, "error"); }
}

async function loadRunTelemetry() {
  const name = document.getElementById("run-select").value;
  if (!name || name.startsWith("No runs")) return;

  document.getElementById("log-status").textContent = "Loading…";
  document.getElementById("log-anomalies").innerHTML = "";
  document.getElementById("log-charts").style.display = "none";

  try {
    const r = await fetch(`/api/runs/${encodeURIComponent(name)}/telemetry`);
    if (!r.ok) { const e = await r.json(); throw new Error(e.detail); }
    const d = await r.json();

    if (!d.rows.length) {
      document.getElementById("log-status").textContent = "Run has no telemetry rows.";
      return;
    }

    renderLogCharts(d.rows, d.mode_changes);
    renderAnomalies(d.rows, d.mode_changes);
    document.getElementById("log-charts").style.display = "";
    document.getElementById("log-status").textContent =
      `Loaded ${d.rows.length} samples from "${name}".`;
  } catch (e) {
    document.getElementById("log-status").textContent = `Error: ${e.message}`;
    consoleLog("Log load error: " + e, "error");
  }
}

function renderLogCharts(rows, modeChanges) {
  const labels    = rows.map(r => r.t_s.toFixed(1));
  const altData   = rows.map(r => r.abs_alt_m);
  const battData  = rows.map(r => r.battery_pct);

  // Vertical line annotations for mode changes
  const annotations = {};
  modeChanges.forEach((mc, i) => {
    const idx = rows.findIndex(r => r.t_s >= mc.t_s);
    if (idx < 0) return;
    annotations[`mode_${i}`] = {
      type: "line", xMin: idx, xMax: idx,
      borderColor: "#d29922", borderWidth: 1, borderDash: [4,4],
      label: { content: mc.mode, enabled: true, position: "start",
               color: "#d29922", font: { size: 10 } },
    };
  });

  const baseOpts = (yLabel) => ({
    animation: false,
    responsive: true,
    maintainAspectRatio: true,
    plugins: {
      legend: { labels: { color: "#8b949e", font: { size: 11 } } },
      tooltip: { backgroundColor: "#161b22", borderColor: "#30363d", borderWidth: 1,
                 titleColor: "#e6edf3", bodyColor: "#8b949e" },
    },
    scales: {
      x: { ticks: { color: "#8b949e", font: { size: 10 }, maxTicksLimit: 10 },
           grid: { color: "#21262d" } },
      y: { title: { display: true, text: yLabel, color: "#8b949e", font: { size: 11 } },
           ticks: { color: "#8b949e" }, grid: { color: "#21262d" } },
    },
  });

  // Destroy old charts if they exist
  if (_logCharts.alt)  { _logCharts.alt.destroy();  _logCharts.alt  = null; }
  if (_logCharts.batt) { _logCharts.batt.destroy(); _logCharts.batt = null; }

  _logCharts.alt = new Chart(document.getElementById("chart-log-alt"), {
    type: "line",
    data: {
      labels,
      datasets: [{
        label: "Abs Altitude (m)", data: altData,
        borderColor: "#58a6ff", backgroundColor: "#58a6ff22",
        borderWidth: 1.5, pointRadius: 0, tension: 0.2, fill: true,
      }],
    },
    options: baseOpts("Alt (m)"),
  });

  _logCharts.batt = new Chart(document.getElementById("chart-log-batt"), {
    type: "line",
    data: {
      labels,
      datasets: [{
        label: "Battery (%)", data: battData,
        borderColor: "#3fb950", backgroundColor: "#3fb95022",
        borderWidth: 1.5, pointRadius: 0, tension: 0.2, fill: true,
      }],
    },
    options: baseOpts("Battery (%)"),
  });

  // Draw mode-change vertical lines manually as annotations on the alt chart
  // (simple overlay using chartjs-plugin-annotation isn't loaded; draw on afterDraw instead)
  _logCharts.alt._modeChanges = modeChanges;
  _logCharts.alt._rows        = rows;
}

function renderAnomalies(rows, modeChanges) {
  const box = document.getElementById("log-anomalies");
  const chips = [];

  // Mode changes
  modeChanges.forEach(mc => {
    chips.push(`<span class="anomaly-badge" style="color:var(--yellow);border-color:rgba(210,153,34,.4);background:rgba(210,153,34,.1)">
      t=${mc.t_s.toFixed(1)}s → ${mc.mode}</span>`);
  });

  // Low battery events
  let lastLow = -999;
  rows.forEach(r => {
    if (r.battery_pct < 20 && r.t_s - lastLow > 10) {
      chips.push(`<span class="anomaly-badge">low batt ${r.battery_pct.toFixed(0)}% @ t=${r.t_s.toFixed(1)}s</span>`);
      lastLow = r.t_s;
    }
  });

  // Sudden altitude drops (>5 m in one sample)
  for (let i = 1; i < rows.length; i++) {
    const drop = rows[i-1].abs_alt_m - rows[i].abs_alt_m;
    if (drop > 5) {
      chips.push(`<span class="anomaly-badge">alt drop ${drop.toFixed(1)}m @ t=${rows[i].t_s.toFixed(1)}s</span>`);
    }
  }

  box.innerHTML = chips.length
    ? `<div style="margin-bottom:6px;font-size:11px;color:var(--text-muted);text-transform:uppercase;letter-spacing:.5px">Detected Events</div>` + chips.join("")
    : "";
}

// ═══════════════════════════════════════════════════════════
// 7. Scenario editor
// ═══════════════════════════════════════════════════════════

async function loadScenarioEditor() {
  const ta = document.getElementById("scenario-editor");
  const st = document.getElementById("scenario-editor-status");
  st.textContent = "Loading…";
  st.style.color = "var(--text-muted)";
  try {
    const r = await fetch("/api/scenarios");
    const d = await r.json();
    ta.value = JSON.stringify(d.scenarios, null, 2);
    st.textContent = `Loaded ${d.scenarios.length} scenario(s). Edit and Save to apply.`;
  } catch (e) {
    st.textContent = "Error loading scenarios: " + e;
    st.style.color = "var(--red)";
  }
}

async function saveScenarios() {
  const ta = document.getElementById("scenario-editor");
  const st = document.getElementById("scenario-editor-status");

  // Validate JSON client-side first
  let parsed;
  try { parsed = JSON.parse(ta.value); }
  catch (e) { st.textContent = "JSON parse error: " + e; st.style.color = "var(--red)"; return; }

  st.textContent = "Saving…";
  st.style.color = "var(--text-muted)";
  try {
    const r = await fetch("/api/scenarios/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ content: ta.value }),
    });
    const d = await r.json();
    if (r.ok) {
      st.textContent = `✓ Saved ${d.count} scenario(s). Reload Mission Control to see updates.`;
      st.style.color = "var(--green)";
      // Refresh scenario dropdowns
      loadScenarios();
    } else {
      st.textContent = "Save failed: " + d.detail;
      st.style.color = "var(--red)";
    }
  } catch (e) {
    st.textContent = "Save error: " + e;
    st.style.color = "var(--red)";
  }
}

async function runFirstScenario() {
  const ta = document.getElementById("scenario-editor");
  let parsed;
  try { parsed = JSON.parse(ta.value); }
  catch (e) { consoleLog("Editor JSON invalid: " + e, "error"); return; }
  if (!parsed.length) { consoleLog("No scenarios in editor", "error"); return; }
  // Save first, then run
  await saveScenarios();
  const name = parsed[0].name;
  consoleLog(`→ Running first scenario from editor: ${name}`, "action");
  await _runScenarioByName(name);
}

// ═══════════════════════════════════════════════════════════
// 8. Failure injection
// ═══════════════════════════════════════════════════════════

async function loadFailureOptions() {
  try {
    const r = await fetch("/api/failures/options");
    const d = await r.json();
    document.getElementById("fail-component").innerHTML =
      d.components.map(c => `<option value="${c}">${c}</option>`).join("");
    document.getElementById("fail-type").innerHTML =
      d.types.map(t => `<option value="${t}">${t}</option>`).join("");
  } catch (e) { consoleLog("Could not load failure options: " + e, "error"); }
}

async function injectFailure() {
  const droneId   = document.getElementById("fail-drone").value;
  const component = document.getElementById("fail-component").value;
  const ftype     = document.getElementById("fail-type").value;
  const instIdx   = parseInt(document.getElementById("fail-inst").value || "0");

  consoleLog(`→ Inject: drone${droneId} ${component}/${ftype} inst=${instIdx}`, "warn");
  try {
    const r = await fetch(`/api/drone/${droneId}/inject`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ component, ftype, instance_idx: instIdx }),
    });
    const d = await r.json();
    consoleLog(
      `${d.accepted ? "✓" : "✗"} Inject ${component}/${ftype}: ${d.accepted ? "accepted" : "rejected"}`,
      d.accepted ? "action" : "error"
    );
  } catch (e) { consoleLog("Inject error: " + e, "error"); }
}

async function clearFailure() {
  const droneId   = document.getElementById("fail-drone").value;
  const component = document.getElementById("fail-component").value;
  const instIdx   = parseInt(document.getElementById("fail-inst").value || "0");

  consoleLog(`→ Clear: drone${droneId} ${component} inst=${instIdx}`, "action");
  try {
    const r = await fetch(`/api/drone/${droneId}/clear`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ component, ftype: "ok", instance_idx: instIdx }),
    });
    const d = await r.json();
    consoleLog(r.ok ? `✓ Cleared ${component}` : `✗ ${d.detail}`, r.ok ? "action" : "error");
  } catch (e) { consoleLog("Clear error: " + e, "error"); }
}

// ═══════════════════════════════════════════════════════════
// 9. Mission control
// ═══════════════════════════════════════════════════════════

async function loadScenarios() {
  try {
    const r = await fetch("/api/scenarios");
    const d = await r.json();
    const sel = document.getElementById("scenario-select");
    sel.innerHTML = d.scenarios.length
      ? d.scenarios.map(s => `<option value="${s.name}">${s.name} (drone ${s.instance})</option>`).join("")
      : `<option disabled>No scenarios found</option>`;
  } catch (e) { consoleLog("Could not load scenarios: " + e, "error"); }
}

async function runScenario() {
  const name = document.getElementById("scenario-select").value;
  if (!name) return;
  await _runScenarioByName(name);
}

async function _runScenarioByName(name) {
  consoleLog(`→ Starting scenario: ${name}`, "action");
  try {
    const r = await fetch("/api/scenario/run", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ scenario_name: name }),
    });
    const d = await r.json();
    if (r.ok) {
      consoleLog(`✓ Scenario '${name}' started — watch Console for events`, "action");
      // Refresh run list after a short delay so the new run shows up
      setTimeout(loadRuns, 5000);
    } else {
      consoleLog(`✗ Scenario failed: ${d.detail}`, "error");
    }
  } catch (e) { consoleLog("Scenario error: " + e, "error"); }
}

// ═══════════════════════════════════════════════════════════
// 10. Console log
// ═══════════════════════════════════════════════════════════

let _consoleDivs = [];
let _lastLogCount = 0;
const MAX_LINES = 400;

function syncServerLogs(logs) {
  if (logs.length <= _lastLogCount) return;
  const newLines = logs.slice(_lastLogCount);
  _lastLogCount = logs.length;
  newLines.forEach(line => {
    let cls = "log-line";
    if (/EVENT|flight_mode|in_air|armed|altitude_band|speed_band/.test(line)) cls += " event";
    else if (/error|Error|ERROR|fail/i.test(line))  cls += " error";
    else if (/warn|WARN|Warning/i.test(line))        cls += " warn";
    else if (/arm|takeoff|land|goto|inject|clear|start/i.test(line)) cls += " action";
    appendConsoleLine(line, cls);
  });
}

function consoleLog(msg, cls = "") {
  appendConsoleLine(msg, "log-line " + cls);
}

function appendConsoleLine(msg, cls) {
  const wrap = document.getElementById("console-log");
  const div  = document.createElement("div");
  div.className = cls;
  div.textContent = msg;
  wrap.appendChild(div);
  _consoleDivs.push(div);
  if (_consoleDivs.length > MAX_LINES) _consoleDivs.shift().remove();
  if (wrap.scrollHeight - wrap.scrollTop - wrap.clientHeight < 80) wrap.scrollTop = wrap.scrollHeight;
}

function clearConsole() {
  document.getElementById("console-log").innerHTML = "";
  _consoleDivs = [];
  _lastLogCount = 0;
}

// ═══════════════════════════════════════════════════════════
// 11. Init
// ═══════════════════════════════════════════════════════════

document.addEventListener("DOMContentLoaded", () => {
  initMap();
  initCharts();
  wsConnect();
  loadFailureOptions();
  loadScenarios();
  loadScenarioEditor();
  loadRuns();
});
