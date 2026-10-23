const SURFACE_COLORS = {
  paved: '#444444', gravel: '#c9a64f', path: '#6aa84f',
  unpaved: '#8b4513', other: '#888888', ferry: '#2a8fe8'
};
let A = null, B = null, results = [], activeRank = 1, info = null;
let currentMode = 'between';

// Basemap style and street/terrain toggle: assets/basemap.js.
const map = new maplibregl.Map({
  container: 'map',
  style: BASEMAP_STYLE,
  center: [8.2, 46.8], zoom: 7
});
setupBasemaps(map);

function setStatus(html) { document.getElementById('status').innerHTML = html; }
function fmtPt(p) { return `${p[1].toFixed(4)}, ${p[0].toFixed(4)}`; }
function fmtAB() {
  document.getElementById('lblA').textContent = A ? 'A: ' + fmtPt(A) : 'A: click on map';
  document.getElementById('lblB').textContent = B ? 'B: ' + fmtPt(B) : 'B: click on map';
  document.getElementById('btnBetween').disabled = !(A && B);
}

const markerA = new maplibregl.Marker({ color: '#2a8fe8' });
const markerB = new maplibregl.Marker({ color: '#e63946' });

map.on('load', async () => {
  // Corridor polygon source — one polygon per ranked route, only the active
  // one is visible. Added before route lines so the line draws on top.
  map.addSource('corridors', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
  map.addLayer({
    id: 'corridor-fill', type: 'fill', source: 'corridors',
    filter: ['==', ['get', 'rank'], 1],
    paint: {
      'fill-color': '#e63946',
      'fill-opacity': 0.08,
      'fill-outline-color': '#e63946'
    }
  });
  map.addLayer({
    id: 'corridor-outline', type: 'line', source: 'corridors',
    filter: ['==', ['get', 'rank'], 1],
    paint: {
      'line-color': '#e63946',
      'line-width': 1,
      'line-opacity': 0.45,
      'line-dasharray': [2, 2]
    }
  });
  // Country borders overlay — Natural Earth admin-0 (1:50m) drawn over the
  // basemap so country lines are clearly visible regardless of which basemap
  // is active.
  map.addSource('borders', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
  map.addLayer({
    id: 'borders-line', type: 'line', source: 'borders',
    paint: {
      'line-color': '#1a1a1a',
      'line-width': 1.6,
      'line-opacity': 0.7
    }
  });
  fetch('https://d2ad6b4ur7yvpq.cloudfront.net/naturalearth-3.3.0/ne_50m_admin_0_boundary_lines_land.geojson')
    .then(r => r.json())
    .then(data => map.getSource('borders').setData(data))
    .catch(() => { /* offline / blocked: silently fall back to basemap-only borders */ });
  // Reference (great-circle) line A→B for the active route — drawn under the
  // routed path so the deviation between intent and reality is visible.
  map.addSource('reference', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
  map.addLayer({
    id: 'reference-line', type: 'line', source: 'reference',
    filter: ['==', ['get', 'rank'], 1],
    paint: {
      'line-color': '#888',
      'line-width': 1.5,
      'line-dasharray': [4, 3],
      'line-opacity': 0.8
    }
  });
  // Intended-shape outline source — added here, but the layer is added below
  // (after the route layers) so the dashed guide is drawn on TOP of the red
  // route line and stays visible.
  map.addSource('shape-guides', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
  // Route lines + endpoint dots.
  map.addSource('routes', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
  // Wide, transparent halo to give clicks generous tolerance even when the
  // visible line is thin. Drawn under the visible line.
  map.addLayer({
    id: 'routes-hit', type: 'line', source: 'routes',
    filter: ['==', ['geometry-type'], 'LineString'],
    paint: { 'line-color': '#000', 'line-width': 18, 'line-opacity': 0 }
  });
  map.addLayer({
    id: 'routes-line', type: 'line', source: 'routes',
    filter: ['==', ['geometry-type'], 'LineString'],
    paint: {
      'line-color': ['case', ['==', ['get', 'rank'], 1], '#e63946', '#f4a261'],
      'line-width': ['case', ['==', ['get', 'rank'], 1], 5, 2.5],
      'line-opacity': ['interpolate', ['linear'], ['get', 'rank'], 1, 1.0, 20, 0.4]
    }
  });
  map.addLayer({
    id: 'routes-end', type: 'circle', source: 'routes',
    filter: ['all', ['==', ['geometry-type'], 'Point'], ['!=', ['get', 'kind'], 'snap']],
    paint: {
      'circle-color': ['case', ['==', ['get', 'rank'], 1], '#e63946', '#f4a261'],
      'circle-radius': ['case', ['==', ['get', 'rank'], 1], 6, 4],
      'circle-stroke-color': '#fff', 'circle-stroke-width': 1.5
    }
  });
  // "Snap dots": small white-ringed dots showing where the router actually
  // started/ended after snapping the requested A/B to the nearest graph node.
  // Only emitted when the snap displacement is large enough to be informative.
  map.addLayer({
    id: 'snap-line', type: 'line', source: 'routes',
    filter: ['==', ['get', 'kind'], 'snap-leg'],
    paint: { 'line-color': '#222', 'line-width': 1, 'line-dasharray': [2, 2], 'line-opacity': 0.6 }
  });
  map.addLayer({
    id: 'snap-dot', type: 'circle', source: 'routes',
    filter: ['==', ['get', 'kind'], 'snap'],
    paint: {
      'circle-color': '#222', 'circle-radius': 3.5,
      'circle-stroke-color': '#fff', 'circle-stroke-width': 1.5
    }
  });
  // Intended-shape outline. Bright cyan dashed, drawn ON TOP of the route so
  // you can see "what we asked for" overlaid on "what we got". Filter to the
  // active rank only — multiple shape outlines on top of each other is noisy.
  map.addLayer({
    id: 'shape-guide-line', type: 'line', source: 'shape-guides',
    filter: ['==', ['get', 'rank'], 1],
    paint: {
      'line-color': '#0096dc',
      'line-width': 2.5,
      'line-dasharray': [2, 2.5],
      'line-opacity': 0.95,
    }
  });
  // Text-mode strokes: each letter's strokes become separate LineStrings.
  // Drawn red and bold. Pen-up connectors between strokes go in a separate
  // source (`text-penups`) and render thinner + grey, so the eye reads the
  // letters as the primary content.
  map.addSource('text-strokes', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
  map.addSource('text-penups',  { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
  map.addLayer({
    id: 'text-penup-line', type: 'line', source: 'text-penups',
    layout: { 'line-cap': 'round', 'line-join': 'round' },
    paint: {
      'line-color': '#888',
      'line-width': 1.5,
      'line-opacity': 0.65,
      'line-dasharray': [2, 3],
    }
  });
  map.addLayer({
    id: 'text-stroke-line', type: 'line', source: 'text-strokes',
    layout: { 'line-cap': 'round', 'line-join': 'round' },
    paint: {
      'line-color': '#e63946',
      'line-width': 4,
      'line-opacity': 0.95,
    }
  });
  map.on('mousemove', 'routes-hit', () => { map.getCanvas().style.cursor = 'pointer'; });
  map.on('mouseleave', 'routes-hit', () => { map.getCanvas().style.cursor = ''; });

  try {
    const r = await fetch('/api/info');
    info = await r.json();
    if (info.bbox) {
      map.fitBounds([[info.bbox[0], info.bbox[1]], [info.bbox[2], info.bbox[3]]],
                    { padding: 30, duration: 0 });
      applyRegionDefaults(info.bbox);
    }
    setStatus(`graph ready — ${info.graph_nodes.toLocaleString()} nodes`);
  } catch (e) {
    setStatus('failed to load /api/info');
    return;
  }
  runLink();
});

map.on('click', (e) => {
  // If the click landed on (or near) a route line, treat it as "select that
  // route" instead of placing an A/B marker. The invisible 'routes-hit' layer
  // is 18 px wide so even thin background routes are clickable.
  if (map.getLayer('routes-hit')) {
    const hits = map.queryRenderedFeatures(e.point, { layers: ['routes-hit'] });
    if (hits.length > 0) {
      // If multiple routes overlap at the click, prefer the one whose
      // rendered line is closest (lower rank → bolder, more likely intended).
      hits.sort((x, y) => (+x.properties.rank) - (+y.properties.rank));
      selectRank(+hits[0].properties.rank);
      return;
    }
  }
  // A/B placement is only meaningful in "between" mode — country, shape,
  // and text searches don't use endpoints, so we ignore map clicks there.
  if (currentMode !== 'between') return;
  const p = [e.lngLat.lng, e.lngLat.lat];
  if (!A) { A = p; markerA.setLngLat(p).addTo(map); }
  else if (!B) { B = p; markerB.setLngLat(p).addTo(map); }
  else {
    // Cycle: replace whichever is older.
    A = B; B = p;
    markerA.setLngLat(A); markerB.setLngLat(B);
  }
  fmtAB();
});

function switchMode(mode) {
  currentMode = mode;
  document.getElementById('left').dataset.mode = mode;
  document.querySelectorAll('.tabs button').forEach(b =>
    b.classList.toggle('active', b.dataset.mode === mode));
  document.getElementById('panelBetween').classList.toggle('active', mode === 'between');
  document.getElementById('panelCountry').classList.toggle('active', mode === 'country');
  document.getElementById('panelShape').classList.toggle('active', mode === 'shape');
  document.getElementById('panelText').classList.toggle('active', mode === 'text');
  saveParams();
}
document.querySelectorAll('.tabs button').forEach(b => {
  b.addEventListener('click', () => switchMode(b.dataset.mode));
});

function setRailCollapsed(side, collapsed) {
  const el = document.getElementById(side === 'left' ? 'left' : 'right');
  const btn = document.getElementById(side === 'left' ? 'leftToggle' : 'rightToggle');
  el.classList.toggle('collapsed', collapsed);
  if (btn) {
    if (side === 'left') btn.textContent = collapsed ? '›' : '‹';
    else                 btn.textContent = collapsed ? '‹' : '›';
  }
  try { localStorage.setItem(`crowfly:rail:${side}`, collapsed ? '1' : '0'); } catch (_) {}
  // Let the rail finish its width transition before maplibre recomputes.
  setTimeout(() => map.resize(), 200);
}
document.getElementById('leftToggle').addEventListener('click', () => {
  setRailCollapsed('left', !document.getElementById('left').classList.contains('collapsed'));
});
document.getElementById('rightToggle').addEventListener('click', () => {
  setRailCollapsed('right', !document.getElementById('right').classList.contains('collapsed'));
});
try {
  if (localStorage.getItem('crowfly:rail:left')  === '1') setRailCollapsed('left',  true);
  if (localStorage.getItem('crowfly:rail:right') === '1') setRailCollapsed('right', true);
} catch (_) {}

document.getElementById('clearAB').addEventListener('click', () => {
  A = null; B = null;
  markerA.remove(); markerB.remove();
  fmtAB();
});

// Points in each route's elevation chart. Climb totals always come from the
// full 50 m terrain profile, and the ascent term is always part of ranking.
const CHART_POINTS = 60;

// The activity select encodes API modes, plus ":paved" for paved-only.
function activity() {
  const [modes, flag] = document.getElementById('optActivity').value.split(':');
  return { modes, paved_only: flag === 'paved' };
}

function commonPayload() {
  // optWidth is in km on the form; the API takes metres.
  const widthKm = parseFloat(document.getElementById('optWidth').value);
  return {
    ...activity(),
    width_m: Math.round((Number.isFinite(widthKm) ? widthKm : 0) * 1000),
    elevation_samples: CHART_POINTS,
  };
}

// Persist the sidebar inputs to localStorage so a refresh keeps the user's
// last-used settings. The schema is just `id ↦ value` (checked state for
// checkboxes and radios).
const PARAM_IDS = [
  'optActivity','optWidth',
  'optCrossing','optAnyLength','optLength',
  'optShape','optPerim',
  'optText','optLetterKm','optFont',
];
const isToggle = el => el.type === 'checkbox' || el.type === 'radio';
function setInput(id, value) {
  const el = document.getElementById(id);
  if (!el) return;
  if (isToggle(el)) el.checked = value === true || value === '1';
  else {
    el.value = value;
    // A value the select doesn't offer leaves it empty: keep the default.
    if (el.tagName === 'SELECT' && el.value === '') el.selectedIndex = 0;
  }
  el.dispatchEvent(new Event('input', { bubbles: true }));
}
// Map settings from before the simplified UI (saved state or old links) onto
// the current inputs: modes + paved-only → activity, the border checkbox and
// distance range → crossing / any-line length.
function migrateParams(get) {
  const on = x => x === true || x === '1';
  const out = {};
  const modes = get('optModes');
  if (modes != null && get('optActivity') == null) {
    out.optActivity = on(get('optPaved')) && modes.includes('bike') ? 'bike:paved' : modes;
  }
  const border = get('optBorder');
  if (border != null && get('optCrossing') == null) {
    out.optCrossing = on(border);
    out.optAnyLength = !on(border);
    const lo = parseFloat(get('optDmin')), hi = parseFloat(get('optDmax'));
    if (Number.isFinite(lo) && Number.isFinite(hi)) out.optLength = String(Math.round((lo + hi) / 2));
  }
  return out;
}
// Inputs restored from saved state; region defaults leave these alone.
const restored = new Set();
function loadParams() {
  try {
    const raw = localStorage.getItem('crowfly:params');
    if (!raw) return;
    const v = JSON.parse(raw);
    Object.assign(v, migrateParams(k => v[k]));
    for (const id of PARAM_IDS) {
      if (!(id in v)) continue;
      setInput(id, v[id]);
      restored.add(id);
    }
    if (typeof v.__mode === 'string' && ['between','country','shape','text'].includes(v.__mode)) {
      switchMode(v.__mode);
    }
    // Migration: corridor used to be stored in metres (e.g. 8000). Now it's
    // km (e.g. 8). Anything ≥ 200 is almost certainly the old metres value.
    const wEl = document.getElementById('optWidth');
    if (wEl && parseFloat(wEl.value) >= 200) {
      wEl.value = (parseFloat(wEl.value) / 1000).toString();
    }
    saveParams();
  } catch (_) { /* ignore corrupt state */ }
}
function saveParams() {
  const v = { __mode: currentMode };
  for (const id of PARAM_IDS) {
    const el = document.getElementById(id);
    if (el) v[id] = isToggle(el) ? el.checked : el.value;
  }
  try { localStorage.setItem('crowfly:params', JSON.stringify(v)); } catch (_) {}
}
// Size-dependent defaults (line length, loop length, letter height) suit the
// loaded region instead of Switzerland: a 150 km line or 15 km letters can't
// fit Liechtenstein. Only inputs the user hasn't set are touched.
function applyRegionDefaults(bbox) {
  const [w, s, e, n] = bbox;
  const midLat = (s + n) / 2 * Math.PI / 180;
  const widthKm = (e - w) * 111.32 * Math.cos(midLat), heightKm = (n - s) * 110.57;
  const [small, large] = [Math.min(widthKm, heightKm), Math.max(widthKm, heightKm)];
  const round5 = x => Math.max(5, Math.round(x / 5) * 5);
  const defaults = {
    optLength: round5(0.4 * large),
    optPerim: Math.min(50, round5(0.6 * small)),
    optLetterKm: Math.min(15, Math.max(2, Math.round(small / 4))),
  };
  for (const [id, value] of Object.entries(defaults)) {
    if (!restored.has(id)) setInput(id, String(value));
  }
}
loadParams();
for (const id of PARAM_IDS) {
  const el = document.getElementById(id);
  if (el) el.addEventListener('change', saveParams);
}

// Permalinks: a finished search writes its mode, the inputs it ran with and
// the server's seed into the URL hash. Opening that URL (or refreshing)
// re-runs the same search; same seed + same graph gives identical results,
// since candidate sampling is the only randomness.
const LINK_PARAMS = {
  between: ['optActivity','optWidth'],
  country: ['optActivity','optWidth','optCrossing','optAnyLength','optLength'],
  shape:   ['optActivity','optWidth','optShape','optPerim'],
  text:    ['optActivity','optText','optLetterKm','optFont'],
};
const RUN_BUTTON = { between: 'btnBetween', country: 'btnCountry', shape: 'btnShapeSearch', text: 'btnText' };
// Seed from an opened permalink, consumed by the next search request.
let linkSeed = null;

function fmtLink(p) { return `${p[1].toFixed(5)},${p[0].toFixed(5)}`; }
function parseLink(s) {
  const [lat, lon] = (s || '').split(',').map(Number);
  return Number.isFinite(lat) && Number.isFinite(lon) ? [lon, lat] : null;
}

// Capture the search's inputs when it starts, so edits made while it runs
// don't leak into its link.
function linkSnapshot() {
  const q = new URLSearchParams({ mode: currentMode });
  for (const id of LINK_PARAMS[currentMode]) {
    const el = document.getElementById(id);
    if (el) q.set(id, isToggle(el) ? (el.checked ? '1' : '0') : el.value);
  }
  if (currentMode === 'between' && A && B) {
    q.set('a', fmtLink(A));
    q.set('b', fmtLink(B));
  }
  return q;
}

function takeLinkSeed() {
  const s = linkSeed;
  linkSeed = null;
  return s === null ? {} : { seed: s };
}

// Record a finished search in the URL; returns a status suffix to copy it.
function publishLink(q, seed) {
  if (seed !== undefined && seed !== null) q.set('seed', String(seed));
  const hash = '#' + q.toString();
  history.replaceState(null, '', hash);
  return ` · <a href="${hash}" class="copy-link" title="Copy a link that re-runs this exact search">copy link</a>`;
}

// Apply the permalink in the URL hash (mode, inputs, A/B, seed) and start
// its search. No-op when the hash isn't a crowfly link.
function runLink() {
  const q = new URLSearchParams(location.hash.slice(1));
  const mode = q.get('mode');
  if (!Object.hasOwn(LINK_PARAMS, mode)) return;
  switchMode(mode);
  const values = Object.fromEntries(q);
  Object.assign(values, migrateParams(k => values[k]));
  for (const id of LINK_PARAMS[mode]) {
    if (id in values) setInput(id, values[id]);
  }
  saveParams();
  if (mode === 'between') {
    A = parseLink(q.get('a'));
    B = parseLink(q.get('b'));
    if (A) markerA.setLngLat(A).addTo(map); else markerA.remove();
    if (B) markerB.setLngLat(B).addTo(map); else markerB.remove();
    fmtAB();
  }
  const seed = q.get('seed');
  linkSeed = mode !== 'between' && /^\d+$/.test(seed || '') ? Number(seed) : null;
  document.getElementById(RUN_BUTTON[mode]).click();
}
// Pasting a link into an open tab only changes the hash.
window.addEventListener('hashchange', runLink);

document.getElementById('status').addEventListener('click', async (e) => {
  const a = e.target.closest('.copy-link');
  if (!a) return;
  e.preventDefault();
  try {
    await navigator.clipboard.writeText(location.href);
    a.textContent = 'link copied';
  } catch (_) {
    a.textContent = 'copy failed — use the address bar';
  }
});

let inFlight = null;
let progressTimer = null;
function showProgress(label) {
  document.getElementById('progressRow').classList.remove('hidden');
  document.getElementById('progressLabel').textContent = label;
  const bar = document.getElementById('progressBar');
  bar.removeAttribute('value');
  bar.max = 1;
}
function hideProgress() {
  document.getElementById('progressRow').classList.add('hidden');
  if (progressTimer) { clearInterval(progressTimer); progressTimer = null; }
}
function startProgressPoll() {
  if (progressTimer) clearInterval(progressTimer);
  const startedAt = performance.now();
  progressTimer = setInterval(async () => {
    try {
      const r = await fetch('/api/progress').then(r => r.json());
      if (!r.active) return;
      const bar = document.getElementById('progressBar');
      const elapsed = ((performance.now() - startedAt) / 1000).toFixed(0);
      if (r.total > 0) {
        bar.max = r.total;
        bar.value = r.done;
        const label = ({
          'country':      'evaluating candidates',
          'shape-search': 'evaluating shapes',
          'between':      'finding alternatives',
          'text':         'evaluating placements',
        })[r.kind] || r.kind;
        document.getElementById('progressLabel').textContent =
          `${label}: ${r.done}/${r.total} · ${elapsed}s`;
      }
    } catch (_) { /* ignore poll failures */ }
  }, 400);
}
async function postJson(path, body) {
  if (inFlight) inFlight.abort();
  const ac = new AbortController();
  inFlight = ac;
  setBusy(true);
  try {
    const r = await fetch(path, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body), signal: ac.signal
    });
    if (!r.ok) {
      const j = await r.json().catch(() => ({ error: 'request failed' }));
      throw new Error(j.error || ('HTTP ' + r.status));
    }
    return await r.json();
  } finally {
    if (inFlight === ac) {
      inFlight = null;
      setBusy(false);
    }
    hideProgress();
  }
}

function setBusy(busy) {
  document.getElementById('btnBetween').disabled = busy || !(A && B);
  document.getElementById('btnCountry').disabled = busy;
  document.getElementById('btnShapeSearch').disabled = busy;
  const tb = document.getElementById('btnText');
  if (tb) tb.disabled = busy;
  document.getElementById('btnCancel').style.display = busy ? '' : 'none';
}

// Aborting the fetch only stops the browser waiting; /api/cancel stops the
// server's search too. (Starting a new search also cancels the old one.)
function cancelServerSearch() {
  navigator.sendBeacon('/api/cancel');
}
document.getElementById('btnCancel').addEventListener('click', () => {
  if (inFlight) {
    inFlight.abort();
    cancelServerSearch();
    setStatus('cancelled');
    document.getElementById('cards').innerHTML = '<div class="empty">cancelled</div>';
    hideProgress();
  }
});
// Closing or reloading the tab mid-search shouldn't leave the server busy.
window.addEventListener('pagehide', () => { if (inFlight) cancelServerSearch(); });

function isAbort(e) { return e && (e.name === 'AbortError' || e.message === 'cancelled'); }

document.getElementById('btnBetween').addEventListener('click', async () => {
  if (!A || !B) return;
  const link = linkSnapshot();
  setStatus('<span class="spinner"></span> finding alternatives between A and B…');
  document.getElementById('cards').innerHTML = '<div class="empty">searching…</div>';
  showProgress('finding alternatives…');
  startProgressPoll();
  try {
    const j = await postJson('/api/between', {
      ...commonPayload(),
      start: [A[1], A[0]],
      end:   [B[1], B[0]],
    });
    renderResults(j.results, j.failures);
    setStatus(`got ${j.results.length} alternatives` + publishLink(link));
  } catch (e) {
    if (isAbort(e)) return;
    setStatus('error: ' + e.message);
    document.getElementById('cards').innerHTML = `<div class="empty">${e.message}</div>`;
  }
});

document.getElementById('btnText').addEventListener('click', async () => {
  const text = document.getElementById('optText').value;
  if (!text.trim()) {
    setStatus('error: text is empty');
    return;
  }
  const letterKm = parseFloat(document.getElementById('optLetterKm').value);
  const teaser = text.length > 32 ? text.slice(0, 32) + '…' : text;
  const link = linkSnapshot();
  setStatus(`<span class="spinner"></span> searching for a place to write "${teaser}"…`);
  document.getElementById('cards').innerHTML = '<div class="empty">searching placements…</div>';
  showProgress('searching placements…');
  startProgressPoll();
  try {
    // Line wrapping, the per-stroke detour cap and the placement budget use
    // the server's defaults (wrap fits the region's width).
    const j = await postJson('/api/text', {
      ...activity(),
      elevation_samples: CHART_POINTS,
      text,
      letter_height_m: Math.round((Number.isFinite(letterKm) ? letterKm : 15) * 1000),
      font: document.getElementById('optFont').value,
      ...takeLinkSeed(),
    });
    renderResults(j.results, j.failures);
    const linkHtml = publishLink(link, j.seed);
    if (j.results.length === 0) {
      const m = j.text_render_meta || {};
      const evald = m.evaluated || 0;
      const hint = m.hint || 'try bigger letters or a shorter message';
      setStatus(`no viable placement after ${evald} candidates — ${hint}` + linkHtml);
    } else {
      const r = j.results[0];
      const tr = r.text_render || {};
      const total = tr.stroke_count || 0;
      const miss = tr.missing_glyphs || 0;
      const evald = tr.candidates_evaluated || 0;
      const detPct = tr.detour_pct;
      const parts = [`placed "${teaser}" (${total} stroke${total === 1 ? '' : 's'}, ${detPct ?? 0}% detour)`];
      parts.push(`evaluated ${evald} candidates`);
      if (miss) parts.push(`${miss} non-ASCII glyph${miss === 1 ? '' : 's'} dropped`);
      setStatus(parts.join(' · ') + linkHtml);
    }
  } catch (e) {
    if (isAbort(e)) return;
    setStatus('error: ' + e.message);
    document.getElementById('cards').innerHTML = `<div class="empty">${e.message}</div>`;
  }
});

document.getElementById('btnShapeSearch').addEventListener('click', async () => {
  const kind = document.getElementById('optShape').value;
  const perim = +document.getElementById('optPerim').value;
  const link = linkSnapshot();
  setStatus(`<span class="spinner"></span> searching for ${kind}s of ${perim} km across the country…`);
  document.getElementById('cards').innerHTML = '<div class="empty">searching…</div>';
  showProgress(`searching ${kind}s…`);
  startProgressPoll();
  try {
    const j = await postJson('/api/shape-search', {
      ...commonPayload(),
      kind,
      perimeter_km: perim,
      ...takeLinkSeed(),
    });
    renderResults(j.results, j.failures);
    setStatus(`got ${j.results.length} ${kind}${j.results.length === 1 ? '' : 's'}`
              + publishLink(link, j.seed));
  } catch (e) {
    if (isAbort(e)) return;
    setStatus('error: ' + e.message);
    document.getElementById('cards').innerHTML = `<div class="empty">${e.message}</div>`;
  }
});

document.getElementById('btnCountry').addEventListener('click', async () => {
  const link = linkSnapshot();
  setStatus('<span class="spinner"></span> sampling diverse country-wide lines…');
  document.getElementById('cards').innerHTML = '<div class="empty">searching…</div>';
  showProgress('preparing candidates…');
  startProgressPoll();
  try {
    // Crossings are bounded by the country; other lines get ±20 % around the
    // requested length.
    const crossing = document.getElementById('optCrossing').checked;
    const lengthKm = parseFloat(document.getElementById('optLength').value);
    const span = crossing ? { border_to_border: true }
      : Number.isFinite(lengthKm) && lengthKm > 0
        ? { distance_min_km: 0.8 * lengthKm, distance_max_km: 1.2 * lengthKm } : {};
    const j = await postJson('/api/country', {
      ...commonPayload(),
      ...span,
      ...takeLinkSeed(),
    });
    renderResults(j.results, j.failures);
    setStatus(`got ${j.results.length} country-wide suggestions` + publishLink(link, j.seed));
  } catch (e) {
    if (isAbort(e)) return;
    setStatus('error: ' + e.message);
    document.getElementById('cards').innerHTML = `<div class="empty">${e.message}</div>`;
  }
});

function haversineMeters(a, b) {
  const R = 6371008.8;
  const toRad = d => d * Math.PI / 180;
  const phi1 = toRad(a[1]); const phi2 = toRad(b[1]);
  const dphi = toRad(b[1] - a[1]); const dlam = toRad(b[0] - a[0]);
  const h = Math.sin(dphi/2)**2 + Math.cos(phi1) * Math.cos(phi2) * Math.sin(dlam/2)**2;
  return 2 * R * Math.asin(Math.sqrt(h));
}

function renderResults(rs, failures) {
  results = rs;
  // Build GeoJSON.
  const feats = [];
  const corridorFeats = [];
  const refFeats = [];
  const shapeGuideFeats = [];
  const textStrokeFeats = [];
  const textPenUpFeats = [];
  for (const r of rs) {
    const isShape = !!r.shape; // shapes are closed loops, no start/end/corridor/reference
    const isText = r.kind === 'text' && r.text_render && Array.isArray(r.text_render.letters);
    if (isText) {
      // Each letter contributes one LineString per Hershey stroke; pen-ups
      // (within a letter and between adjacent letters) go to a separate
      // source so they can render thinner/dashed.
      for (const letter of r.text_render.letters) {
        for (const stroke of (letter.strokes || [])) {
          if (stroke && stroke.length >= 2) {
            textStrokeFeats.push({
              type: 'Feature',
              properties: { rank: r.rank, ch: letter.char || '' },
              geometry: { type: 'LineString', coordinates: stroke }
            });
          }
        }
      }
      for (const pu of (r.text_render.pen_ups || [])) {
        if (pu && pu.length >= 2) {
          textPenUpFeats.push({
            type: 'Feature',
            properties: { rank: r.rank },
            geometry: { type: 'LineString', coordinates: pu }
          });
        }
      }
      continue; // skip generic LineString / endpoints / corridor below
    }
    feats.push({
      type: 'Feature',
      properties: { rank: r.rank },
      geometry: { type: 'LineString', coordinates: r.coords }
    });
    if (isShape && r.shape.vertices && r.shape.vertices.length >= 2) {
      shapeGuideFeats.push({
        type: 'Feature',
        properties: { rank: r.rank },
        geometry: { type: 'LineString', coordinates: r.shape.vertices }
      });
    }
    if (!isShape) {
      feats.push({
        type: 'Feature',
        properties: { rank: r.rank, kind: 'end' },
        geometry: { type: 'Point', coordinates: r.end }
      });
      feats.push({
        type: 'Feature',
        properties: { rank: r.rank, kind: 'start' },
        geometry: { type: 'Point', coordinates: r.start }
      });
      // Snap-displacement indicators: the router started/ended at coords[0] /
      // coords[last], not at the requested r.start / r.end. Show only when
      // the gap is larger than 50 m so we don't clutter the map for clean
      // snaps. Skipped for shapes since "requested start" = center is just
      // a placement detail, not a routing endpoint.
      if (r.coords.length >= 2) {
        const snapStart = r.coords[0];
        const snapEnd = r.coords[r.coords.length - 1];
        if (haversineMeters(r.start, snapStart) > 50) {
          feats.push({
            type: 'Feature', properties: { rank: r.rank, kind: 'snap' },
            geometry: { type: 'Point', coordinates: snapStart }
          });
          feats.push({
            type: 'Feature', properties: { rank: r.rank, kind: 'snap-leg' },
            geometry: { type: 'LineString', coordinates: [r.start, snapStart] }
          });
        }
        if (haversineMeters(r.end, snapEnd) > 50) {
          feats.push({
            type: 'Feature', properties: { rank: r.rank, kind: 'snap' },
            geometry: { type: 'Point', coordinates: snapEnd }
          });
          feats.push({
            type: 'Feature', properties: { rank: r.rank, kind: 'snap-leg' },
            geometry: { type: 'LineString', coordinates: [r.end, snapEnd] }
          });
        }
      }
      if (r.corridor && r.corridor.length >= 4) {
        corridorFeats.push({
          type: 'Feature',
          properties: { rank: r.rank },
          geometry: { type: 'Polygon', coordinates: [r.corridor] }
        });
      }
      refFeats.push({
        type: 'Feature',
        properties: { rank: r.rank },
        geometry: { type: 'LineString', coordinates: [r.start, r.end] }
      });
    }
  }
  map.getSource('routes').setData({ type: 'FeatureCollection', features: feats });
  map.getSource('corridors').setData({ type: 'FeatureCollection', features: corridorFeats });
  map.getSource('reference').setData({ type: 'FeatureCollection', features: refFeats });
  map.getSource('shape-guides').setData({ type: 'FeatureCollection', features: shapeGuideFeats });
  if (map.getSource('text-strokes')) {
    map.getSource('text-strokes').setData({ type: 'FeatureCollection', features: textStrokeFeats });
  }
  if (map.getSource('text-penups')) {
    map.getSource('text-penups').setData({ type: 'FeatureCollection', features: textPenUpFeats });
  }
  // Split into primary (route fit inside the requested corridor) and
  // alternatives (search had to widen). Cards keep their server-assigned
  // ranks so the existing map filters / GPX downloads still work.
  const primary = [], alts = [];
  for (const r of rs) {
    const used = Number.isFinite(r.corridor_used_m) ? r.corridor_used_m : 0;
    const reqW = Number.isFinite(r.width_m) ? r.width_m : Infinity;
    if (used <= reqW * 1.05) primary.push(r); else alts.push(r);
  }

  let summary = '';
  if (rs.length) {
    let totalAsc = 0, withElev = 0;
    for (const r of rs) {
      if (r.elevation && Number.isFinite(r.elevation.ascent_m)) {
        totalAsc += r.elevation.ascent_m;
        withElev += 1;
      }
    }
    if (withElev > 0) {
      summary = `<div class="empty" style="text-align:left;color:#444;padding:10px 4px;">` +
                `Σ elevation gain across ${withElev} route${withElev === 1 ? '' : 's'}: ` +
                `<b>↗ ${Math.round(totalAsc).toLocaleString()} m</b></div>`;
    }
  }

  let html = summary;
  if (primary.length) {
    html += `<div class="section-head">Routes <span class="muted">(${primary.length})</span></div>`;
    html += primary.map(cardHtml).join('');
  } else if (rs.length) {
    html += `<div class="empty" style="color:#7a5800;background:#fff3cd;
              border:1px solid #ffe28a;border-radius:6px;padding:10px;">
              No route fit inside your requested corridor — showing
              looser-corridor alternatives instead.</div>`;
  }
  if (alts.length) {
    html += `<details class="alts" ${primary.length ? '' : 'open'}>
      <summary>Looser-corridor alternatives <span class="muted">(${alts.length})</span></summary>
      ${alts.map(cardHtml).join('')}
    </details>`;
  }
  document.getElementById('cards').innerHTML = rs.length
    ? html
    : failuresHtml(failures);
  // Wire card events.
  document.querySelectorAll('.card').forEach(card => {
    const rank = +card.dataset.rank;
    card.addEventListener('click', (e) => {
      if (e.target.tagName === 'BUTTON') return;
      selectRank(rank);
    });
    card.querySelector('.dl').addEventListener('click', (e) => {
      e.stopPropagation();
      downloadGpx(rank);
    });
    card.querySelector('.view').addEventListener('click', (e) => {
      e.stopPropagation();
      selectRank(rank);
    });
  });
  // Fit map to all results.
  if (rs.length) {
    const b = new maplibregl.LngLatBounds();
    rs.forEach(r => r.coords.forEach(c => b.extend(c)));
    map.fitBounds(b, { padding: 60, duration: 700 });
    selectRank(1);
  }
}

function failuresHtml(f) {
  if (!f || !f.total) {
    return '<div class="empty">no viable routes — try a wider corridor or another activity</div>';
  }
  // Diagnostic strings for each bucket: short label + a one-line hint of what
  // to change. Order matters — most actionable first.
  const items = [];
  const push = (n, label, hint) => { if (n > 0) items.push({ n, label, hint }); };
  push(f.snap_too_far,      'endpoints too far from any road',
       'move the endpoints, widen the corridor, or pick an activity with more ways (hike opens up paths)');
  push(f.corridor_violated, 'best route bulged outside the corridor',
       'widen the corridor — these would be reachable with a wider band');
  push(f.no_route,          'no path within the corridor',
       'mountains or lakes in the way — widen the corridor, or pick an activity with more ways (gravel / MTB opens trails)');
  push(f.too_winding,       'a side of the shape had to detour too far',
       'try a wider corridor, a different length, or an activity with more ways');
  push(f.no_go,             'route crossed a no-go zone',
       'remove the matching no-go file or move endpoints around it');
  push(f.too_short,         'route shorter than the requested length',
       'ask for a shorter line');
  push(f.too_long,          'route longer than the requested length',
       'ask for a longer line, or widen the corridor so the route needs fewer detours');
  push(f.other,             'other rejection',
       'check server logs for the exact reason');

  const list = items.map(it =>
    `<li><b>${it.n}</b> ${it.label}<br>
     <span style="color:#666;font-size:11px;">→ ${it.hint}</span></li>`
  ).join('');
  return `<div class="empty failures">
    <h4><span>⚠</span> No viable routes (${f.total} candidates rejected)</h4>
    <ul>${list}</ul>
    <div class="hint">The biggest bucket usually points at the right knob to turn.</div>
  </div>`;
}

function cardHtml(r) {
  const detour = r.ref_km > 0 ? (100 * (r.real_km / r.ref_km - 1)) : 0;
  const surfaceTotal = r.surface.reduce((a, x) => a + x.km, 0) || 1;
  const segs = r.surface.map(s =>
    `<span class="seg" title="${s.label} ${s.km.toFixed(1)} km" style="width:${(100*s.km/surfaceTotal).toFixed(2)}%;background:${SURFACE_COLORS[s.label]||'#888'}"></span>`
  ).join('');
  let elev = '';
  if (r.elevation && r.elevation.samples_m && r.elevation.samples_m.length > 1) {
    const e = r.elevation;
    elev = `<div class="elev-row"><span>↗ ${Math.round(e.ascent_m)} m</span>` +
           `<span>↘ ${Math.round(e.descent_m)} m</span>` +
           `<span>max ${Math.round(e.max_m)} m</span>` +
           `<span>min ${Math.round(e.min_m)} m</span></div>` +
           sparklineSvg(e.samples_m, 360, 50);
  }
  const ferryTxt = r.ferry_km > 0.05 ? ` · ferry ${r.ferry_km.toFixed(1)} km` : '';
  // Persistence indicator: when the search loop had to widen the corridor to
  // find this route, surface that here so the user knows it's looser than
  // requested (and can tighten manually if they want).
  let widenBadge = '';
  if (Number.isFinite(r.corridor_used_m) && Number.isFinite(r.width_m)
      && r.corridor_used_m > r.width_m * 1.05) {
    const askKm = (r.width_m / 1000).toFixed(1);
    const usedKm = (r.corridor_used_m / 1000).toFixed(1);
    widenBadge = ` <span title="search widened from ${askKm} km to find a viable line"
      style="background:#fff3cd;color:#7a5800;border:1px solid #ffe28a;
      border-radius:3px;padding:1px 6px;font-size:11px;">↔ corridor ${usedKm} km</span>`;
  }
  // Text-mode header replaces bearing/max-dev with the message + stroke
  // counts. ref_km / real_km still apply (ideal-vs-routed total length).
  let textBadge = '', textRow = '', headerLeft;
  if (r.kind === 'text' && r.text_render) {
    const t = r.text || '';
    const shown = t.length > 48 ? t.slice(0, 48) + '…' : t;
    const missing = r.text_render.missing_glyphs || 0;
    const penUps = r.text_render.pen_up_count || 0;
    const safe = shown.replace(/&/g,'&amp;').replace(/</g,'&lt;');
    textBadge = `<span style="background:#eaf3ff;color:#143b75;border:1px solid #b8d4f6;
      border-radius:3px;padding:1px 6px;font-size:11px;">"${safe}"</span>`;
    const sCount = r.text_render.stroke_count || 0;
    const sKm = r.text_render.stroke_km;
    const puKm = r.text_render.pen_up_km;
    const parts = [`${sCount} letter stroke${sCount===1?'':'s'}`];
    if (Number.isFinite(sKm)) parts.push(`${sKm.toFixed(1)} km drawn`);
    if (penUps) parts.push(`+ ${penUps} pen-up${penUps===1?'':'s'}${Number.isFinite(puKm)?` (${puKm.toFixed(1)} km)`:''}`);
    if (missing) parts.push(`${missing} non-ASCII glyph${missing===1?'':'s'} dropped`);
    textRow = `<div class="row" style="color:#444;">${parts.join(' · ')}</div>`;
    headerLeft = textBadge;
  } else {
    headerLeft = `${Math.round(r.bearing_deg)}°`;
  }
  const metricsRow = (r.kind === 'text')
    ? `<div class="row">${r.ref_km.toFixed(1)} → <b>${r.real_km.toFixed(1)} km</b>
        (${detour>=0?'+':''}${detour.toFixed(0)}% detour)${ferryTxt}</div>`
    : r.shape
      ? `<div class="row">${r.ref_km.toFixed(1)} → <b>${r.real_km.toFixed(1)} km</b>
        (${detour>=0?'+':''}${detour.toFixed(0)}%) · worst leg +${Math.round(r.shape.max_leg_detour_pct || 0)}%${ferryTxt}</div>`
      : `<div class="row">${r.ref_km.toFixed(1)} → <b>${r.real_km.toFixed(1)} km</b>
        (${detour>=0?'+':''}${detour.toFixed(0)}%) · max dev ${Math.round(r.max_dev_m)} m${ferryTxt}</div>`;
  return `<div class="card" data-rank="${r.rank}">
    <div class="hd"><b>#${r.rank}</b> &nbsp; ${headerLeft}${widenBadge}
      <span class="score">score ${r.score.toFixed(1)}</span></div>
    ${textRow}
    ${metricsRow}
    <div class="bar-wrap"><div class="bar">${segs}</div></div>
    <div class="swatches">
      ${r.surface.map(s => `<span><i style="background:${SURFACE_COLORS[s.label]||'#888'}"></i>${s.label} ${s.km.toFixed(1)}</span>`).join('')}
    </div>
    ${elev}
    <div class="actions-card"><button class="view">view</button>
      <button class="dl">⬇ GPX</button></div>
  </div>`;
}

function sparklineSvg(samples, w, h) {
  const n = samples.length;
  let mn = Infinity, mx = -Infinity;
  for (const s of samples) { if (s < mn) mn = s; if (s > mx) mx = s; }
  const span = Math.max(1, mx - mn);
  const xs = samples.map((_, i) => i / (n - 1) * w);
  const ys = samples.map(s => h - ((s - mn) / span) * (h - 4) - 2);
  const pts = xs.map((x, i) => `${x.toFixed(1)},${ys[i].toFixed(1)}`).join(' ');
  let area = `M${xs[0].toFixed(1)},${h} `;
  for (let i = 0; i < n; i++) area += `L${xs[i].toFixed(1)},${ys[i].toFixed(1)} `;
  area += `L${xs[n-1].toFixed(1)},${h} Z`;
  return `<svg class="spark" viewBox="0 0 ${w} ${h}" preserveAspectRatio="none"
    xmlns="http://www.w3.org/2000/svg">
    <path d="${area}" fill="rgba(230,57,70,0.18)"/>
    <polyline points="${pts}" fill="none" stroke="#e63946" stroke-width="1.6"/>
    </svg>`;
}

function selectRank(rank) {
  activeRank = rank;
  document.querySelectorAll('.card').forEach(c =>
    c.classList.toggle('active', +c.dataset.rank === rank));
  // Scroll the active card into view if the user just clicked the map.
  const activeCard = document.querySelector(`.card[data-rank="${rank}"]`);
  if (activeCard) activeCard.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
  if (map.getLayer('routes-line')) {
    map.setPaintProperty('routes-line', 'line-color',
      ['case', ['==', ['get', 'rank'], rank], '#e63946', '#f4a261']);
    map.setPaintProperty('routes-line', 'line-width',
      ['case', ['==', ['get', 'rank'], rank], 5, 2.5]);
    map.setPaintProperty('routes-end', 'circle-color',
      ['case', ['==', ['get', 'rank'], rank], '#e63946', '#f4a261']);
    map.setPaintProperty('routes-end', 'circle-radius',
      ['case', ['==', ['get', 'rank'], rank], 6, 4]);
  }
  if (map.getLayer('corridor-fill')) {
    map.setFilter('corridor-fill', ['==', ['get', 'rank'], rank]);
    map.setFilter('corridor-outline', ['==', ['get', 'rank'], rank]);
  }
  if (map.getLayer('shape-guide-line')) {
    map.setFilter('shape-guide-line', ['==', ['get', 'rank'], rank]);
  }
  if (map.getLayer('reference-line')) {
    map.setFilter('reference-line', ['==', ['get', 'rank'], rank]);
  }
  const r = results.find(x => x.rank === rank);
  if (r) {
    const b = new maplibregl.LngLatBounds();
    r.coords.forEach(c => b.extend(c));
    map.fitBounds(b, { padding: 80, duration: 700 });
  }
}

function xmlEscape(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;')
    .replace(/>/g, '&gt;').replace(/"/g, '&quot;');
}
function slug(s) {
  return s.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-+|-+$/g, '').slice(0, 32);
}

// Track name (shown in GPS device / app lists) and download filename.
function gpxNames(r) {
  const km = Math.round(r.real_km);
  if (r.kind === 'text') {
    const t = (r.text || '').trim();
    return { name: `crowfly "${t}" (${km} km)`, file: `crowfly-text-${slug(t) || 'message'}.gpx` };
  }
  if (r.shape) {
    return { name: `crowfly ${r.shape.kind} #${r.rank} (${km} km)`,
             file: `crowfly-${slug(r.shape.kind)}-${r.rank}-${km}km.gpx` };
  }
  const deg = Math.round(r.bearing_deg);
  return { name: `crowfly line #${r.rank}: ${deg}°, ${km} km`,
           file: `crowfly-line-${r.rank}-${deg}deg-${km}km.gpx` };
}

// One continuous <trkseg>: `coords` is already in ride order for every
// result kind (text mode interleaves strokes and pen-ups server-side).
// Elevations come from the server's DEM; without one the track simply has
// no <ele>. The metadata link is the permalink that reproduces the search.
async function downloadGpx(rank) {
  const r = results.find(x => x.rank === rank);
  if (!r) return;
  let ele = null;
  try {
    const res = await fetch('/api/elevations', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ coords: r.coords }),
    });
    if (res.ok) ele = (await res.json()).ele;
  } catch (_) { /* export without elevation */ }
  const pts = r.coords.map((c, i) => {
    const z = ele && Number.isFinite(ele[i]) ? `<ele>${ele[i].toFixed(1)}</ele>` : '';
    return `      <trkpt lat="${(+c[1]).toFixed(7)}" lon="${(+c[0]).toFixed(7)}">${z}</trkpt>`;
  }).join('\n');
  const { name, file } = gpxNames(r);
  const gpx = `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="crowfly" xmlns="http://www.topografix.com/GPX/1/1">
  <metadata>
    <name>${xmlEscape(name)}</name>
    <link href="${xmlEscape(location.href)}"><text>crowfly search</text></link>
  </metadata>
  <trk><name>${xmlEscape(name)}</name>
    <trkseg>
${pts}
    </trkseg>
  </trk>
</gpx>
`;
  const blob = new Blob([gpx], { type: 'application/gpx+xml' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = file; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 500);
}

fmtAB();
