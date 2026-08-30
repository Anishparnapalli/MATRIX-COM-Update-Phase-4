"use strict";
/* ═══════════════════════════════════════════════════════════════════
 *  M.A.T.R.I.X. Communication Website — app.js  (Phase 3 — Card redesign)
 * ═══════════════════════════════════════════════════════════════════
 *
 *  This file implements the three-card redesign specified in:
 *    - Card_1_Robotic_Dashboard_to_Python_Bridge.txt
 *    - Card_2___Python_Bridge__Gateway_Pro.txt
 *    - CARD_3___QNX_ROBOTIC_ARM__QNX_RTOS.txt
 *
 *  Everything rendered here is derived from REAL envelopes broadcast by
 *  bridge.py on the observability channel (:8766). Nothing is simulated:
 *
 *   - Card 1's 3D robot is driven only by real qnx_to_bridge 'angles'
 *     telemetry (procedural Three.js geometry — no OBJ/STL assets are
 *     available to this page, so a clean hierarchical box-arm is built
 *     here, per the Card 1 spec's "otherwise create a clean procedural
 *     3D robotic arm" allowance).
 *   - Card 2's outbound/inbound lanes light up only on real traffic in
 *     that direction; RX/TX/clients/errors are real counters.
 *   - Card 3's thread states (RECV/MOTION/SAFETY) are *inferred* from
 *     real, already-observable bridge state transitions:
 *       - RECV: flashes RECEIVING right when the bridge dispatches a
 *         bridge_to_qnx envelope (that IS what RECV thread receives —
 *         bridge.py already knows exactly what it put on the wire).
 *       - MOTION: RUNNING while a dispatched sequence hasn't reached
 *         seq_complete/emergency yet (same logic bridge.py itself uses
 *         for state.motion_active).
 *       - SAFETY: MONITORING/EMERGENCY straight from the real
 *         emergency/emergency_cleared events.
 *     QNX's C firmware does not currently publish raw internal thread
 *     telemetry (THREAD_STATUS:... messages), so these are protocol-
 *     level inferences from real events — not fabricated/random
 *     blinking. This is called out explicitly in the UI copy and in
 *     the delivery notes.
 *
 *  OBSERVABILITY_TOKEN — same shared-secret model as the robotic
 *  dashboard's CONTROL_TOKEN. See PROJECT_DESCRIPTION.md Section 5.
 */
const OBSERVABILITY_TOKEN = "83lpr5SKUB8jJa1kiU1_yevaeUCWpJMfqTJChdE55pM";

const GATEWAY_HOST = "localhost";
const GATEWAY_PORT = 8766;

// ── State ──────────────────────────────────────────────────────────
const MAX_PACKETS = 300;
let packets = [];          // ALL envelopes, newest first (unfiltered — Card 2's "everything")
let selectedPacketId = null;
let paused = false;
let currentDetailCard = null;   // null | 'card1' | 'card2' | 'card3'

const stats = {
  total: 0,
  byCategory: {},
  sizeSum: 0,
  windowTimestamps: [],
};

let pendingCommand = null;
const timelineEntries = [];

// Topology: connection state is REAL (from explicit status events, never
// inferred from traffic). Activity is a separate, short-lived
// "traffic seen recently on this hop" indicator.
const topo = {
  qnxOnline: null, brOnline: false, gwOnline: false, gwConnecting: false,
  lastQnxTrafficAt: 0, lastBrTrafficAt: 0, lastGwTrafficAt: 0,
};

let ws = null;
let authOk = false;
let retryTimer = null;

// ═══════════════════════════════════════════════════════════════════
//  CARD 1 STATE — Robotic Dashboard → Bridge  (+ real 3D robot)
// ═══════════════════════════════════════════════════════════════════
const NEUTRAL_ANGLES = [90, 90, 90, 90, 90, 0];

const c1 = {
  count: 0, mode: 'MANUAL', cmdCount: 0, seqCount: 0, safetyCount: 0,
  // Real robot state — the ONLY source of truth is qnx_to_bridge 'angles'
  // telemetry. targetAngles is what QNX most recently reported; the two
  // rendered scenes each keep their own smoothed "current" array so the
  // motion always visually settles on the real value (Card 1 spec §5).
  targetAngles: [...NEUTRAL_ANGLES],
  haveRealTelemetry: false,
  lastTelemetryAt: null,
  motionCycling: false,      // mirrors card3's motion-thread inference
  history: [],               // bounded action history (browser_to_bridge)
  historyIdSeq: 1,
  selectedHistoryId: null,
  payloadOpen: false,
};
const C1_HISTORY_MAX = 60;

// ═══════════════════════════════════════════════════════════════════
//  CARD 2 STATE — Python Bridge (bidirectional journey)
// ═══════════════════════════════════════════════════════════════════
const c2 = {
  rx: 0, tx: 0, errors: 0, clients: 0,
  lastOutAt: 0, lastInAt: 0,
};

// ═══════════════════════════════════════════════════════════════════
//  CARD 3 STATE — QNX RTOS threads (inferred from real bridge events)
// ═══════════════════════════════════════════════════════════════════
const c3 = {
  recvState: 'WAITING',        // WAITING | RECEIVING
  lastCommandToQnx: null,      // {desc, lines, at}
  motionState: 'IDLE',         // IDLE | CYCLING | LOCKED
  poseIdx: null, totalPoses: null, cycleNum: null,
  safetyState: 'MONITORING',   // MONITORING | EMERGENCY
  lastSafetyEvent: null,
  lastQnxEvent: null,          // description string for the overview card
  telemetryTimestamps: [],
  selectedThread: 'motion',
  events: [],                  // bounded discrete QNX events (no raw angle spam)
  selectedEventId: null,
  eventFilter: 'all',
};
const C3_EVENTS_MAX = 60;

let _recvFlashTimer = null;

// ═══════════════════════════════════════════════════════════════════
//  3D ROBOT — procedural hierarchical arm (Card 1 spec §2)
// ═══════════════════════════════════════════════════════════════════
function createArmScene(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 50);

  scene.add(new THREE.AmbientLight(0x304060, 0.9));
  const dl = new THREE.DirectionalLight(0xffffff, 1.05); dl.position.set(2.5, 4, 3); scene.add(dl);
  const rim = new THREE.PointLight(0x00d4ff, 1.4, 12); rim.position.set(-2.5, 3, -2); scene.add(rim);
  const warm = new THREE.PointLight(0xffaa00, 0.35, 10); warm.position.set(2, 1, 2.5); scene.add(warm);

  // Floor ring for spatial reference (no full floor plane — keep it light)
  const ringGeo = new THREE.RingGeometry(1.15, 1.25, 48);
  const ringMat = new THREE.MeshBasicMaterial({ color: 0x0a84ff, side: THREE.DoubleSide, transparent: true, opacity: 0.35 });
  const ring = new THREE.Mesh(ringGeo, ringMat); ring.rotation.x = -Math.PI / 2; scene.add(ring);
  const grid = new THREE.GridHelper(3, 10, 0x0f1a2e, 0x0f1a2e); scene.add(grid);

  const matLink = new THREE.MeshStandardMaterial({ color: 0x1a6cf5, roughness: 0.35, metalness: 0.6 });
  const matJoint = new THREE.MeshStandardMaterial({ color: 0x0f1a2e, roughness: 0.4, metalness: 0.7 });
  const matEnd = new THREE.MeshStandardMaterial({ color: 0xffaa00, roughness: 0.3, metalness: 0.5 });
  const matGrip = new THREE.MeshStandardMaterial({ color: 0xb0c0d8, roughness: 0.35, metalness: 0.55 });

  const base = new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.5, 0.22, 24), matJoint);
  base.position.y = 0.11;
  scene.add(base);

  // J0 — Base Yaw (Y axis)
  const j0 = new THREE.Group(); j0.position.set(0, 0.22, 0); scene.add(j0);
  const j0joint = new THREE.Mesh(new THREE.CylinderGeometry(0.24, 0.24, 0.2, 20), matJoint); j0joint.position.y = 0.1; j0.add(j0joint);
  const link1 = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.75, 0.2), matLink); link1.position.y = 0.2 + 0.375; j0.add(link1);

  // J1 — Shoulder (Z axis)
  const j1 = new THREE.Group(); j1.position.set(0, 0.2 + 0.75, 0); j0.add(j1);
  const j1joint = new THREE.Mesh(new THREE.SphereGeometry(0.135, 16, 16), matJoint); j1.add(j1joint);
  const link2 = new THREE.Mesh(new THREE.BoxGeometry(0.17, 0.65, 0.17), matLink); link2.position.y = 0.325; j1.add(link2);

  // J2 — Elbow (Z axis)
  const j2 = new THREE.Group(); j2.position.set(0, 0.65, 0); j1.add(j2);
  const j2joint = new THREE.Mesh(new THREE.SphereGeometry(0.11, 16, 16), matJoint); j2.add(j2joint);
  const link3 = new THREE.Mesh(new THREE.BoxGeometry(0.135, 0.52, 0.135), matLink); link3.position.y = 0.26; j2.add(link3);

  // J3 — Wrist Pitch (Z axis)
  const j3 = new THREE.Group(); j3.position.set(0, 0.52, 0); j2.add(j3);
  const j3joint = new THREE.Mesh(new THREE.SphereGeometry(0.085, 14, 14), matJoint); j3.add(j3joint);
  const link4 = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.24, 0.1), matLink); link4.position.y = 0.12; j3.add(link4);

  // J4 — Wrist Roll (Y axis)
  const j4 = new THREE.Group(); j4.position.set(0, 0.24, 0); j3.add(j4);
  const j4joint = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.1, 12), matJoint); j4.add(j4joint);

  // J5 — Gripper
  const gripBase = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.05, 0.12), matEnd); gripBase.position.y = 0.08; j4.add(gripBase);
  const gripGroup = new THREE.Group(); gripGroup.position.y = 0.11; j4.add(gripGroup);
  const jawL = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.14, 0.09), matGrip);
  const jawR = jawL.clone();
  jawL.position.set(-0.05, 0.07, 0); jawR.position.set(0.05, 0.07, 0);
  gripGroup.add(jawL); gripGroup.add(jawR);

  function applyAngles(a) {
    const D = Math.PI / 180;
    j0.rotation.y = (a[0] - 90) * D;
    j1.rotation.z = (a[1] - 90) * D;
    j2.rotation.z = -(a[2] - 90) * D;
    j3.rotation.z = (a[3] - 90) * D;
    j4.rotation.y = (a[4] - 90) * D;
    const spread = Math.max(0, Math.min(1, a[5] / 100)) * 0.045;
    jawL.position.x = -0.05 - spread;
    jawR.position.x = 0.05 + spread;
  }
  applyAngles(NEUTRAL_ANGLES);

  function setFrameColor(emergency) {
    const c = emergency ? 0xff2244 : 0x1a6cf5;
    matLink.color.setHex(c);
  }

  const CAMERA_PRESETS = {
    reset: { pos: [1.9, 1.75, 2.6], target: [0, 0.75, 0] },
    front: { pos: [0, 1.1, 3.0], target: [0, 0.9, 0] },
    side: { pos: [3.0, 1.1, 0], target: [0, 0.9, 0] },
    top: { pos: [0, 3.2, 0.01], target: [0, 0.6, 0] },
  };

  camera.position.set(...CAMERA_PRESETS.reset.pos);
  camera.lookAt(...CAMERA_PRESETS.reset.target);

  return {
    renderer, scene, camera, applyAngles, setFrameColor,
    pivots: { j0, j1, j2, j3, j4 },
    CAMERA_PRESETS,
    orbit: null,
  };
}

function attachOrbitControls(armObj) {
  if (typeof THREE.OrbitControls !== 'function') return null;
  const orbit = new THREE.OrbitControls(armObj.camera, armObj.renderer.domElement);
  orbit.enableDamping = true;
  orbit.dampingFactor = 0.08;
  orbit.target.set(...armObj.CAMERA_PRESETS.reset.target);
  orbit.update();
  armObj.orbit = orbit;
  return orbit;
}

function resizeArmScene(armObj, canvas) {
  const w = canvas.clientWidth || 200, h = canvas.clientHeight || 150;
  armObj.renderer.setSize(w, h, false);
  armObj.camera.aspect = w / Math.max(1, h);
  armObj.camera.updateProjectionMatrix();
}

// Overview (compact, no orbit — static camera, per "no decorative
// oscillation" rule) and Detail (larger, orbit-enabled) scenes.
let c1Overview = null;
let c1Detail = null;
let c1OverviewCurrent = [...NEUTRAL_ANGLES];
let c1DetailCurrent = [...NEUTRAL_ANGLES];

function initC1Overview() {
  const canvas = document.getElementById('c1-robot-canvas');
  if (!canvas || c1Overview) return;
  c1Overview = createArmScene(canvas);
  const ro = new ResizeObserver(() => resizeArmScene(c1Overview, canvas));
  ro.observe(canvas.parentElement);
  resizeArmScene(c1Overview, canvas);
}

function initC1Detail() {
  const canvas = document.getElementById('c1-robot-canvas-lg');
  if (!canvas || c1Detail) return;
  c1Detail = createArmScene(canvas);
  attachOrbitControls(c1Detail);
  const ro = new ResizeObserver(() => resizeArmScene(c1Detail, canvas));
  ro.observe(canvas.parentElement);
  resizeArmScene(c1Detail, canvas);
}

function c1SetCamera(name) {
  if (!c1Detail) return;
  const p = c1Detail.CAMERA_PRESETS[name] || c1Detail.CAMERA_PRESETS.reset;
  c1Detail.camera.position.set(...p.pos);
  if (c1Detail.orbit) { c1Detail.orbit.target.set(...p.target); c1Detail.orbit.update(); }
  else c1Detail.camera.lookAt(...p.target);
}

// Robot connection/telemetry state → badge (Card 1 spec §4/§6)
function robotBadgeState() {
  if (topo.qnxOnline === false) {
    return { cls: 'disc', text: 'QNX DISCONNECTED — LAST VALID POSE FROZEN' };
  }
  if (!c1.haveRealTelemetry) {
    return { cls: '', text: 'WAITING FOR ROBOT STATE' };
  }
  if (c1.motionCycling) {
    return { cls: 'live', text: 'LIVE — RTOS CYCLING' };
  }
  const ageS = c1.lastTelemetryAt ? (Date.now() - c1.lastTelemetryAt) / 1000 : null;
  return { cls: '', text: `CONNECTED — LAST POSE ${ageS != null ? ageS.toFixed(0) + 's ago' : ''}`.trim() };
}

function renderRobotBadges() {
  const st = robotBadgeState();
  ['c1-robot-badge', 'c1-robot-badge-lg'].forEach(id => {
    const el = document.getElementById(id);
    if (!el) return;
    el.textContent = st.text;
    el.className = 'c1-robot-badge' + (st.cls ? ' ' + st.cls : '');
  });
}

// Shared render loop for both robot scenes — smooths toward the real
// target (interpolation for visual smoothness only, per Card 1 spec §5).
const ROBOT_LERP = 0.14;
function lerpAngles(cur, target) {
  for (let i = 0; i < 6; i++) cur[i] += (target[i] - cur[i]) * ROBOT_LERP;
}

function robotAnimTick() {
  requestAnimationFrame(robotAnimTick);
  const target = c1.targetAngles;
  const emergency = c3.safetyState === 'EMERGENCY';

  if (c1Overview) {
    lerpAngles(c1OverviewCurrent, target);
    c1Overview.applyAngles(c1OverviewCurrent);
    c1Overview.setFrameColor(emergency);
    c1Overview.scene.rotation.y += 0.0015; // gentle fixed turntable, not "random motion"
    c1Overview.renderer.render(c1Overview.scene, c1Overview.camera);
  }
  if (c1Detail) {
    lerpAngles(c1DetailCurrent, target);
    c1Detail.applyAngles(c1DetailCurrent);
    c1Detail.setFrameColor(emergency);
    if (c1Detail.orbit) c1Detail.orbit.update();
    c1Detail.renderer.render(c1Detail.scene, c1Detail.camera);
  }
  updateJointHudText(c1OverviewCurrent, false);
  if (currentDetailCard === 'card1') updateJointHudText(c1DetailCurrent, true);
  renderRobotBadges();
}

function updateJointHudText(angles, isLarge) {
  const names = ['j0', 'j1', 'j2', 'j3', 'j4'];
  if (!isLarge) {
    names.forEach((id, i) => {
      const el = document.getElementById('c1-' + id);
      if (el) { el.textContent = angles[i].toFixed(1) + '°'; el.classList.toggle('live', c1.haveRealTelemetry); }
    });
    const g = document.getElementById('c1-j5');
    if (g) { g.textContent = Math.round(angles[5]) + '%'; g.classList.toggle('live', c1.haveRealTelemetry); }
  } else {
    const wrap = document.getElementById('c1-joint-hud-lg');
    if (!wrap) return;
    const LABELS = ['Base Yaw', 'Shoulder', 'Elbow', 'Wrist Pitch', 'Wrist Roll'];
    let html = '';
    LABELS.forEach((lbl, i) => {
      html += `<div class="jhf-cell"><span class="jhf-lbl">J${i} ${lbl}</span><span class="jhf-val${c1.haveRealTelemetry ? ' live' : ''}">${angles[i].toFixed(1)}°</span></div>`;
    });
    html += `<div class="jhf-cell"><span class="jhf-lbl">J5 Gripper</span><span class="jhf-val${c1.haveRealTelemetry ? ' live' : ''}">${Math.round(angles[5])}%</span></div>`;
    wrap.innerHTML = html;
  }
}
requestAnimationFrame(robotAnimTick);


// ═══════════════════════════════════════════════════════════════════
//  CONNECTION + AUTH
// ═══════════════════════════════════════════════════════════════════
function connect(){
  setGwStatus("connecting");
  setAuthStatus("—", null);

  const url = `wss://${GATEWAY_HOST}:${GATEWAY_PORT}`;
  try{
    ws = new WebSocket(url);
  } catch(e){
    log("WS construction failed: " + e.message);
    scheduleRetry();
    return;
  }

  ws.onopen = () => {
    setGwStatus("on");
    setAuthStatus("AUTHENTICATING", "warn");
    ws.send(JSON.stringify({ type: "auth", token: OBSERVABILITY_TOKEN }));
  };

  ws.onmessage = (evt) => {
    let msg;
    try{ msg = JSON.parse(evt.data); } catch(e){ return; }

    if(msg.type === "auth_result"){
      if(msg.ok){
        authOk = true;
        setAuthStatus("AUTHENTICATED", "on");
        toast("Authenticated — receiving real MATRIX traffic");
        if(retryTimer){ clearTimeout(retryTimer); retryTimer=null; }
      } else {
        authOk = false;
        setAuthStatus("REJECTED", "off");
        toast("Auth REJECTED — check OBSERVABILITY_TOKEN in app.js");
      }
      return;
    }

    if(msg.type === "event_replay"){
      (msg.events || []).forEach(env => handleEnvelope(env, true));
      toast(`Caught up on ${((msg.events)||[]).length} recent event(s)`);
      return;
    }

    if(msg.type === "test_lab_status"){
      renderTestLabStatus(msg.fault, msg.ids);
      return;
    }
    if(msg.type === "test_lab_ack"){
      toast(msg.detail || (msg.ok ? "Test Lab command applied" : "Test Lab command rejected"));
      return;
    }

    handleEnvelope(msg, false);
  };

  ws.onclose = () => {
    setGwStatus("off");
    setAuthStatus(authOk ? "DISCONNECTED" : "—", "off");
    authOk = false;
    topo.gwOnline = false;
    topo.gwConnecting = false;
    topo.qnxOnline = null;
    topo.brOnline = false;
    renderTopology();
    scheduleRetry();
  };

  ws.onerror = () => { /* onclose follows */ };
}

function scheduleRetry(){
  if(retryTimer) return;
  topo.gwConnecting = true;
  renderTopology();
  retryTimer = setTimeout(()=>{ retryTimer=null; connect(); }, 3000);
}

function setGwStatus(state){
  const dot = document.getElementById('dot-gw');
  const lbl = document.getElementById('lbl-gw');
  topo.gwOnline = (state === "on");
  topo.gwConnecting = (state === "connecting");
  if(state === "on"){ dot.className='dot on'; lbl.textContent='ONLINE'; }
  else if(state === "connecting"){ dot.className='dot warn'; lbl.textContent='CONNECTING'; }
  else { dot.className='dot'; lbl.textContent='OFFLINE'; }
  renderTopology();
}

function setAuthStatus(text, cls){
  const dot = document.getElementById('dot-auth');
  const lbl = document.getElementById('lbl-auth');
  lbl.textContent = text;
  dot.className = 'dot' + (cls ? ' '+cls : '');
}

function setQnxStatus(online){
  topo.qnxOnline = online;
  const dot = document.getElementById('dot-qnx');
  const lbl = document.getElementById('lbl-qnx');
  if(online === true){ dot.className='dot on'; lbl.textContent='ONLINE'; }
  else if(online === false){ dot.className='dot'; lbl.textContent='OFFLINE'; }
  else { dot.className='dot warn'; lbl.textContent='UNKNOWN'; }
  renderTopology();
  renderC3Overview();
}
function setBrStatus(online){
  topo.brOnline = online;
  const dot = document.getElementById('dot-br');
  const lbl = document.getElementById('lbl-br');
  dot.className = 'dot' + (online ? ' on' : '');
  lbl.textContent = online ? 'ONLINE' : 'OFFLINE';
  renderTopology();
}

// ═══════════════════════════════════════════════════════════════════
//  ENVELOPE HANDLING — routes into stats, the three cards, and lists
// ═══════════════════════════════════════════════════════════════════
function handleEnvelope(env, isReplay){
  if(!env || typeof env.packet_id === 'undefined') return;
  const replayed = !!(isReplay || env.replayed);
  const p = env.payload || {};

  // ── Real connection-status events (Item 1) ──
  if(env.category === 'system_status'){
    if(typeof p.qnx_connected !== 'undefined') setQnxStatus(!!p.qnx_connected);
    if(p.type === 'browser_status'){
      setBrStatus(!!p.browser_connected);
      if(typeof p.count === 'number') c2.clients = p.count;
    }
  }

  // ── Traffic/activity tracking ──
  if(!replayed){
    const now = Date.now();
    if(env.direction === 'qnx_to_bridge' || env.direction === 'bridge_to_qnx'){
      topo.lastQnxTrafficAt = now; firePulse('qnx-gw');
    }
    if(env.direction === 'bridge_to_browser' || env.direction === 'browser_to_bridge'){
      topo.lastBrTrafficAt = now; firePulse('gw-br');
    }
    topo.lastGwTrafficAt = now;
  }

  if(!replayed){
    updateStats(env);
    updateLatency(env);
  }

  // ── Card-specific routing ──
  if(!replayed) updateCard1(env);
  updateCard2Counters(env, replayed);
  if(!replayed) updateCard2Lanes(env);
  updateCard3(env, replayed);

  // ── Packet store (used by Card 2's transaction list + inspector) ──
  packets.unshift(env);
  if(packets.length > MAX_PACKETS) packets.pop();
  if(!paused && currentDetailCard === 'card2') renderC2TxList();

  renderStats();
}

// ═══════════════════════════════════════════════════════════════════
//  CARD 1 — Robotic Dashboard → Bridge (+ live 3D robot)
// ═══════════════════════════════════════════════════════════════════
function updateCard1(env){
  // Real robot state comes ONLY from qnx_to_bridge telemetry — never
  // from what the dashboard *asked* QNX to do (Card 1 spec §22).
  if(env.direction === 'qnx_to_bridge'){
    const p = env.payload || {};
    if(p.type === 'angles' && Array.isArray(p.angles)){
      c1.targetAngles = p.angles.slice(0,6);
      c1.haveRealTelemetry = true;
      c1.lastTelemetryAt = Date.now();
    }
    if(p.type === 'seq_complete') c1.motionCycling = false;
    if(p.type === 'emergency') c1.motionCycling = false;
    return;
  }

  if(env.direction !== 'browser_to_bridge') return;
  const p = env.payload || {};

  // Bounded action history (Card 1 spec §12) — meaningful dashboard
  // actions only, not raw telemetry.
  const entry = {
    id: c1.historyIdSeq++,
    env, ts: Date.now(),
  };
  c1.history.unshift(entry);
  if(c1.history.length > C1_HISTORY_MAX) c1.history.pop();

  c1.count++;
  if(env.category === 'sequence' && (p.type === 'rtos_sequence' || p.type === 'send_sequence')){
    c1.seqCount++;
    c1.motionCycling = true;
    renderCard1PosePreview(p);
  } else if(env.category === 'emergency'){
    c1.safetyCount++;
  } else {
    c1.cmdCount++;
  }
  if(p.type === 'set_mode' && p.mode) c1.mode = p.mode.toUpperCase();
  if(p.type === 'stop_cycle'){ /* motionCycling cleared on real seq_complete, not here */ }

  document.getElementById('c1-latest').textContent = describe(env);
  document.getElementById('c1-latest-time').textContent = new Date().toLocaleTimeString('en-GB',{hour12:false});
  document.getElementById('c1-cmd-count').textContent = c1.cmdCount;
  document.getElementById('c1-seq-count').textContent = c1.seqCount;
  document.getElementById('c1-safety-count').textContent = c1.safetyCount;

  if(currentDetailCard === 'card1') renderC1History();
}

function renderCard1PosePreview(p){
  const wrap = document.getElementById('c1-pose-preview');
  if(!wrap) return;
  const seq = p.sequence;
  if(!Array.isArray(seq) || seq.length === 0){ wrap.style.display = 'none'; return; }
  wrap.style.display = 'flex';
  wrap.innerHTML = seq.map((pose, i) => {
    const j0 = Array.isArray(pose) && pose.length ? pose[0] : null;
    return `<div class="pose-chip"><span class="pose-chip-lbl">P${i+1}</span><span class="pose-chip-val">${j0!=null ? j0.toFixed(0)+'°' : '—'}</span></div>`;
  }).join('');
}

const C1_FILTERS = [
  { id: 'all', label: 'ALL' },
  { id: 'sequences', label: 'SEQUENCES' },
  { id: 'commands', label: 'COMMANDS' },
  { id: 'safety', label: 'SAFETY' },
  { id: 'mode', label: 'MODE' },
];
let c1ActiveFilter = 'all';

function _matchesC1Filter(env){
  if(c1ActiveFilter === 'all') return true;
  const p = env.payload || {};
  if(c1ActiveFilter === 'sequences') return env.category === 'sequence';
  if(c1ActiveFilter === 'safety') return env.category === 'emergency';
  if(c1ActiveFilter === 'mode') return p.type === 'set_mode';
  if(c1ActiveFilter === 'commands') return env.category === 'command';
  return true;
}
function setC1Filter(id){ c1ActiveFilter = id; renderC1Subfilters(); renderC1History(); }
function renderC1Subfilters(){
  const wrap = document.getElementById('detail-subfilters-c1');
  if(!wrap) return;
  wrap.innerHTML = `<div class="c1-filter-row">${C1_FILTERS.map(f =>
    `<button class="c1-filter-btn${f.id===c1ActiveFilter?' active':''}" onclick="setC1Filter('${f.id}')">${f.label}</button>`
  ).join('')}</div>`;
}

function renderC1History(){
  const list = document.getElementById('c1-history-list');
  if(!list) return;
  const visible = c1.history.filter(h => _matchesC1Filter(h.env));
  if(visible.length === 0){
    list.innerHTML = '<div class="c1-history-empty">NO DASHBOARD TRANSMISSIONS<br>Send a command or sequence from the robotic dashboard.</div>';
    return;
  }
  list.innerHTML = visible.map(h => {
    const t = new Date(h.ts).toLocaleTimeString('en-GB',{hour12:false});
    return `<div class="c1-history-item${h.id===c1.selectedHistoryId?' selected':''}" onclick="c1SelectHistory(${h.id})">
      <div class="hi-title">${escapeHtml(describe(h.env))}</div>
      <div class="hi-time">${t}</div>
    </div>`;
  }).join('');
}

function c1SelectHistory(id){
  c1.selectedHistoryId = id;
  c1.payloadOpen = false;
  renderC1History();
  renderC1Selected();
}

function renderC1Selected(){
  const body = document.getElementById('c1-selected-body');
  const btn  = document.getElementById('c1-payload-btn');
  const panel = document.getElementById('c1-payload-panel');
  if(!body) return;
  const h = c1.history.find(x => x.id === c1.selectedHistoryId);
  if(!h){
    body.innerHTML = 'Select an action from the history on the left.';
    btn.style.display = 'none'; panel.style.display = 'none';
    return;
  }
  const env = h.env, p = env.payload || {};
  const rows = [
    ['Type', describe(env)],
    ['Direction', 'Dashboard → Python Bridge'],
    ['Transport', env.transport],
    ['Timestamp', new Date(h.ts).toLocaleTimeString('en-GB',{hour12:false})],
  ];
  if(p.total_poses != null) rows.push(['Pose Count', String(p.total_poses)]);
  if(p.pose_duration_ms != null) rows.push(['Duration', p.pose_duration_ms + ' ms / pose']);
  rows.push(['Size', env.size_bytes + ' bytes']);
  body.innerHTML = rows.map(([k,v]) => `<div class="sf-row"><span class="sf-key">${escapeHtml(k)}</span><span class="sf-val">${escapeHtml(String(v))}</span></div>`).join('')
    + `<div class="sf-row"><span class="sf-key">Status</span><span class="sf-val">✓ SENT · ✓ BRIDGE RECEIVED</span></div>`;

  const hasSeq = Array.isArray(p.sequence) && p.sequence.length > 0;
  btn.style.display = hasSeq ? '' : 'none';
  if(!hasSeq){ panel.style.display = 'none'; c1.payloadOpen = false; }
  else {
    btn.textContent = c1.payloadOpen ? 'HIDE PAYLOAD' : 'INSPECT PAYLOAD';
    panel.style.display = c1.payloadOpen ? '' : 'none';
    if(c1.payloadOpen){
      panel.innerHTML = p.sequence.map((pose,i) => {
        const j = pose.map((v,k)=> `J${k} ${v.toFixed(0)}°`).join(' · ');
        return `<div class="c1-payload-pose"><b>POSE ${i+1}</b> — ${j}</div>`;
      }).join('');
    }
  }
}
function c1TogglePayload(){ c1.payloadOpen = !c1.payloadOpen; renderC1Selected(); }


// ═══════════════════════════════════════════════════════════════════
//  CARD 2 — Python Bridge (bidirectional communication journey)
// ═══════════════════════════════════════════════════════════════════
function updateCard2Counters(env, replayed){
  if(replayed) return;
  // RX = what the bridge received on either transport; TX = what the
  // bridge sent out. Both are real, derived from envelope direction —
  // never fabricated packet-processing stages (Card 2 spec §5/§14).
  if(env.direction === 'browser_to_bridge' || env.direction === 'qnx_to_bridge') c2.rx++;
  if(env.direction === 'bridge_to_qnx' || env.direction === 'bridge_to_browser') c2.tx++;
  if(env.category === 'error') c2.errors++;

  document.getElementById('c2-rx').textContent = c2.rx;
  document.getElementById('c2-tx').textContent = c2.tx;
  document.getElementById('c2-clients').textContent = c2.clients;
  document.getElementById('c2-errors').textContent = c2.errors;
}

const LANE_OUT_DIRS = new Set(['browser_to_bridge', 'bridge_to_qnx']);
const LANE_IN_DIRS  = new Set(['qnx_to_bridge', 'bridge_to_browser']);

function updateCard2Lanes(env){
  const now = Date.now();
  if(LANE_OUT_DIRS.has(env.direction)){
    c2.lastOutAt = now;
    animateLane('out');
  }
  if(LANE_IN_DIRS.has(env.direction)){
    c2.lastInAt = now;
    animateLane('in');
  }
}

function animateLane(dir){
  const isOv = true, isDt = currentDetailCard === 'card2';
  const specs = [];
  if(isOv) specs.push({ dotId: dir==='out'?'ov-out-dot':'ov-in-dot', nodeIds: dir==='out'?['ov-out-n0','ov-out-n1','ov-out-n2']:['ov-in-n0','ov-in-n1','ov-in-n2'], xs: dir==='out'?[20,130,240]:[20,130,240], y:23, stateId: dir==='out'?'lane-out-state-ov':'lane-in-state-ov' });
  if(isDt) specs.push({ dotId: dir==='out'?'dt-out-dot':'dt-in-dot', nodeIds: dir==='out'?['dt-out-n0','dt-out-n1','dt-out-n2','dt-out-n3']:['dt-in-n0','dt-in-n1','dt-in-n2','dt-in-n3'], xs: [40,220,420,600], y:35, stateId: dir==='out'?'lane-out-state-detail':'lane-in-state-detail' });

  specs.forEach(spec => {
    const dot = document.getElementById(spec.dotId);
    const stateEl = document.getElementById(spec.stateId);
    if(!dot) return;
    const nodes = spec.nodeIds.map(id => document.getElementById(id)).filter(Boolean);
    dot.style.opacity = '1';
    nodes.forEach(n => n.classList.remove('active'));
    if(stateEl){ stateEl.textContent = 'ACTIVE'; stateEl.classList.add('active'); }
    spec.xs.forEach((x,i) => {
      setTimeout(() => {
        dot.setAttribute('cx', x);
        dot.setAttribute('cy', spec.y);
        if(nodes[i]) nodes[i].classList.add('active');
      }, i*90);
    });
    clearTimeout(dot._fade);
    dot._fade = setTimeout(() => {
      dot.style.opacity = '0';
      nodes.forEach(n => n.classList.remove('active'));
      if(stateEl){ stateEl.textContent = 'IDLE'; stateEl.classList.remove('active'); }
    }, spec.xs.length*90 + 500);
  });
}

const C2_FILTERS = [
  { id: 'all', label: 'ALL' },
  { id: 'outbound', label: 'OUTBOUND' },
  { id: 'inbound', label: 'INBOUND' },
];
let c2ActiveFilter = 'all';
function setC2Filter(id){ c2ActiveFilter = id; renderC2Subfilters(); renderC2TxList(); }
function renderC2Subfilters(){
  const wrap = document.getElementById('detail-subfilters-c2');
  if(!wrap) return;
  wrap.innerHTML = `<div class="c1-filter-row">${C2_FILTERS.map(f =>
    `<button class="c1-filter-btn${f.id===c2ActiveFilter?' active':''}" onclick="setC2Filter('${f.id}')">${f.label}</button>`
  ).join('')}</div>`;
}
function _c2Direction(env){ return LANE_OUT_DIRS.has(env.direction) ? 'OUTBOUND' : (LANE_IN_DIRS.has(env.direction) ? 'INBOUND' : null); }
function renderC2TxList(){
  const list = document.getElementById('c2-tx-list');
  if(!list) return;
  let visible = packets.filter(env => _c2Direction(env) !== null);
  if(c2ActiveFilter !== 'all') visible = visible.filter(env => _c2Direction(env) === c2ActiveFilter.toUpperCase());
  visible = visible.slice(0, 200);
  if(visible.length === 0){
    list.innerHTML = '<div class="c1-history-empty">NO TRANSACTIONS YET</div>';
    return;
  }
  list.innerHTML = visible.map(env => {
    const dir = _c2Direction(env);
    const t = new Date().toLocaleTimeString('en-GB',{hour12:false});
    return `<div class="c2-tx-item${env.packet_id===selectedPacketId?' selected':''}" onclick="c2SelectTx(${env.packet_id})">
      <span class="c2-tx-id">#${env.packet_id}</span>
      <span class="c2-tx-dir ${dir}">${dir}</span>
      <span class="c2-tx-desc">${escapeHtml(describe(env))}</span>
      <span class="c2-tx-time">${env.size_bytes}B</span>
    </div>`;
  }).join('');
}
function c2SelectTx(pid){
  selectedPacketId = pid;
  const env = packets.find(p => p.packet_id === pid);
  if(env) renderInspector(env);
  renderC2TxList();
}

// ═══════════════════════════════════════════════════════════════════
//  CARD 3 — QNX RTOS threads (inferred from real bridge-observable state)
// ═══════════════════════════════════════════════════════════════════
function _c3PushEvent(env){
  const p = env.payload || {};
  if(p.type === 'angles') return; // never a raw telemetry wall (spec §13)
  c3.events.unshift({ id: env.packet_id, env, ts: Date.now() });
  if(c3.events.length > C3_EVENTS_MAX) c3.events.pop();
}

function updateCard3(env, replayed){
  const p = env.payload || {};

  // ── RECV THREAD inference: bridge dispatching a command to QNX IS
  // what the RECV thread receives — bridge.py already knows exactly
  // what it put on the wire (Card 3 spec §7/§15). ──
  if(env.direction === 'bridge_to_qnx'){
    c3.lastCommandToQnx = { desc: describe(env), lines: p.lines || [], at: Date.now() };
    if(!replayed){
      c3.recvState = 'RECEIVING';
      clearTimeout(_recvFlashTimer);
      _recvFlashTimer = setTimeout(() => { c3.recvState = 'WAITING'; renderC3Overview(); if(currentDetailCard==='card3') renderC3Detail(); }, 450);
    }
  }

  if(env.direction === 'qnx_to_bridge'){
    if(!replayed) _c3PushEvent(env);

    if(p.type === 'angles'){
      c3._trackTelemetryRate();
      // still update "last event"? No — telemetry doesn't overwrite the
      // discrete "last QNX event" text (Card 3 spec §13).
    } else {
      c3.lastQnxEvent = describe(env);
      document.getElementById('c3-latest').textContent = c3.lastQnxEvent;
    }

    if(p.type === 'pose_advance'){ c3.poseIdx = p.pose_idx; c3.motionState = 'CYCLING'; }
    if(p.type === 'cycle_done'){ c3.cycleNum = p.cycle_num; }
    if(p.type === 'seq_complete'){ c3.motionState = 'IDLE'; c3.poseIdx = null; }
    if(p.type === 'emergency'){ c3.motionState = 'LOCKED'; c3.safetyState = 'EMERGENCY'; c3.lastSafetyEvent = describe(env); }
    if(p.type === 'emergency_cleared'){ c3.motionState = 'IDLE'; c3.safetyState = 'MONITORING'; c3.lastSafetyEvent = describe(env); }
  }

  if(env.direction === 'browser_to_bridge'){
    if(p.type === 'rtos_sequence' || p.type === 'send_sequence'){
      c3.totalPoses = p.total_poses || (p.sequence ? p.sequence.length : null);
      c3.motionState = 'CYCLING';
    }
    if(p.type === 'emergency'){ c3.safetyState = 'EMERGENCY'; c3.motionState = 'LOCKED'; c3.lastSafetyEvent = describe(env); }
  }

  renderC3Overview();
  if(currentDetailCard === 'card3'){
    renderC3Detail();
    if(!paused && !replayed) renderC3EventList();
  }
}

c3.telemetryTimestamps = [];
c3._trackTelemetryRate = function(){
  const now = performance.now();
  c3.telemetryTimestamps.push(now);
  const cutoff = now - 3000;
  while(c3.telemetryTimestamps.length && c3.telemetryTimestamps[0] < cutoff) c3.telemetryTimestamps.shift();
};
function c3TelemetryHz(){
  if(c3.telemetryTimestamps.length < 2) return null;
  return c3.telemetryTimestamps.length / 3;
}

function _threadCssClass(state){
  if(state === 'RECEIVING' || state === 'CYCLING') return 'busy';
  if(state === 'LOCKED' || state === 'EMERGENCY') return 'danger';
  if(state === 'MONITORING' || state === 'WAITING' || state === 'IDLE') return '';
  return '';
}

function renderC3Overview(){
  // Mini thread cards
  const map = [
    ['th-recv-ov', c3.recvState],
    ['th-motion-ov', c3.motionState],
    ['th-safety-ov', c3.safetyState],
  ];
  map.forEach(([id, state]) => {
    const el = document.getElementById(id);
    if(!el) return;
    el.className = 'thread-mini ' + _threadCssClass(state);
    const txt = el.querySelector('.th-state-txt');
    if(txt) txt.textContent = state;
  });

  document.getElementById('c3-safety-sum').textContent = c3.safetyState === 'EMERGENCY' ? 'EMERGENCY' : 'NORMAL';
  const safetyStat = document.getElementById('c3-safety-sum');
  if(safetyStat) safetyStat.style.color = c3.safetyState === 'EMERGENCY' ? 'var(--red)' : 'var(--green)';
  document.getElementById('c3-tcp-sum').textContent = topo.qnxOnline === true ? 'CONNECTED' : topo.qnxOnline === false ? 'DISCONNECTED' : '—';
  if(c3.lastQnxEvent) document.getElementById('c3-latest').textContent = c3.lastQnxEvent;
}

function c3SelectThread(name){
  c3.selectedThread = name;
  renderC3Detail();
}

function renderC3Detail(){
  const map = [
    ['th-recv-detail', 'recv', c3.recvState],
    ['th-motion-detail', 'motion', c3.motionState],
    ['th-safety-detail', 'safety', c3.safetyState],
  ];
  map.forEach(([id, key, state]) => {
    const el = document.getElementById(id);
    if(!el) return;
    el.className = 'thread-card ' + _threadCssClass(state) + (c3.selectedThread===key?' selected':'');
    const txt = el.querySelector('.th-state-txt');
    if(txt) txt.textContent = state;
  });

  const insp = document.getElementById('c3-thread-inspector');
  const out  = document.getElementById('c3-output-panel');
  if(insp) insp.innerHTML = renderThreadInspectorHtml(c3.selectedThread);
  if(out) out.innerHTML = renderC3OutputHtml();
}

function renderThreadInspectorHtml(which){
  const row = (k,v) => `<div class="ti-row"><span class="ti-key">${escapeHtml(k)}</span><span class="ti-val">${escapeHtml(String(v))}</span></div>`;
  if(which === 'recv'){
    const last = c3.lastCommandToQnx;
    let html = '<b style="color:var(--fg);font-family:var(--head);font-size:13px">RECV THREAD</b>';
    html += row('State', c3.recvState);
    html += row('Scheduling', 'Normal (not SCHED_FIFO)');
    html += row('Role', 'Receives commands from Python Bridge');
    html += row('Input', 'Python Bridge → QNX (TCP)');
    html += row('Responsibility', 'TCP reception + command parsing / controller-state update');
    html += row('Last command', last ? last.desc : 'None yet this session');
    if(last && last.lines && last.lines.length){
      html += `<div class="ti-flow">Wire lines: <span style="color:var(--fg)">${escapeHtml(last.lines.join(' / '))}</span></div>`;
    }
    return html;
  }
  if(which === 'motion'){
    let html = '<b style="color:var(--fg);font-family:var(--head);font-size:13px">MOTION TASK</b>';
    html += row('State', c3.motionState);
    html += row('Scheduler', 'SCHED_FIFO');
    html += row('Priority', '10');
    html += row('Period', '100 ms');
    html += row('Current pose', c3.poseIdx!=null ? `${c3.poseIdx}${c3.totalPoses?'/'+c3.totalPoses:''}` : '—');
    html += row('Current cycle', c3.cycleNum!=null ? `#${c3.cycleNum}` : '—');
    const hz = c3TelemetryHz();
    html += row('Telemetry', hz!=null ? `ACTIVE · ${hz.toFixed(1)} Hz` : 'idle (no active cycle)');
    const steps = ['SEQUENCE READY','POSE EXECUTION','JOINT STATE / POSE_DONE','NEXT POSE','CYCLE_DONE','SEQ_COMPLETE'];
    const activeIdx = c3.motionState === 'CYCLING' ? 2 : (c3.motionState === 'LOCKED' ? -1 : 5);
    html += '<div class="ti-flow">' + steps.map((s,i) => {
      const cls = i < activeIdx ? 'done' : (i === activeIdx ? '' : '');
      return `<div class="ti-flow-step${cls?(' '+cls):''}">${i===activeIdx?'▶ ':'　'}${s}</div>`;
    }).join('') + '</div>';
    return html;
  }
  // safety
  let html = '<b style="color:var(--fg);font-family:var(--head);font-size:13px">SAFETY TASK</b>';
  html += row('State', c3.safetyState);
  html += row('Scheduler', 'SCHED_FIFO');
  html += row('Priority', '20 (HIGHEST)');
  html += row('Latest safety event', c3.lastSafetyEvent || 'None yet this session');
  html += row('QNX → Bridge output', c3.safetyState === 'EMERGENCY' ? 'EMERGENCY_STOP' : 'EMERGENCY_CLEARED (last confirmed)');
  html += `<div class="ti-flow">SAFETY MONITOR<br>　├── NORMAL ${c3.safetyState==='MONITORING'?'◀ current':''}<br>　└── EMERGENCY ${c3.safetyState==='EMERGENCY'?'◀ current → PYTHON BRIDGE':''}</div>`;
  return html;
}

function renderC3OutputHtml(){
  const now = Date.now();
  const recent = (types) => c3.events.length > 0 && types.includes((c3.events[0].env.payload||{}).type) && (now - c3.events[0].ts) < 2500;
  const telemetryActive = c3TelemetryHz() != null;
  const grp = (title, items) => {
    return `<div class="c3-out-group"><div class="c3-out-group-title">${title}</div>` +
      items.map(([label, on]) => `<div class="c3-out-item${on?' recent':''}"><span class="th-dot"></span>${label}</div>`).join('') +
      `</div>`;
  };
  return grp('ROBOT STATE', [[ 'Joint telemetry' + (telemetryActive?' — ACTIVE':''), telemetryActive ]])
    + grp('MOTION EVENTS', [
        ['POSE_DONE', recent(['pose_advance'])],
        ['CYCLE_DONE', recent(['cycle_done'])],
        ['SEQ_COMPLETE', recent(['seq_complete'])],
      ])
    + grp('SAFETY EVENTS', [
        ['EMERGENCY_STOP', c3.safetyState === 'EMERGENCY'],
        ['EMERGENCY_CLEARED', recent(['emergency_cleared'])],
      ]);
}

const C3_FILTERS = [
  { id: 'all', label: 'ALL' },
  { id: 'telemetry', label: 'TELEMETRY' },
  { id: 'pose', label: 'POSE' },
  { id: 'cycle', label: 'CYCLE' },
  { id: 'safety', label: 'SAFETY' },
  { id: 'system', label: 'SYSTEM' },
];
function setC3Filter(id){ c3.eventFilter = id; renderC3Subfilters(); renderC3EventList(); }
function renderC3Subfilters(){
  const wrap = document.getElementById('detail-subfilters-c3');
  if(!wrap) return;
  wrap.innerHTML = `<div class="c1-filter-row">${C3_FILTERS.map(f =>
    `<button class="c1-filter-btn${f.id===c3.eventFilter?' active':''}" onclick="setC3Filter('${f.id}')">${f.label}</button>`
  ).join('')}</div>`;
}
function _matchesC3EventFilter(env){
  const p = env.payload || {};
  switch(c3.eventFilter){
    case 'pose': return p.type === 'pose_advance';
    case 'cycle': return p.type === 'cycle_done' || p.type === 'seq_complete';
    case 'safety': return env.category === 'emergency';
    case 'system': return env.category === 'system_status' || p.type === 'watchdog';
    default: return true;
  }
}
function renderC3EventList(){
  const list = document.getElementById('c3-event-list');
  if(!list) return;

  if(c3.eventFilter === 'telemetry'){
    const angles = c1.targetAngles;
    const hz = c3TelemetryHz();
    list.innerHTML = `<div class="c3-telemetry-current">` +
      ['J0','J1','J2','J3','J4'].map((tag,i)=>`<div class="c3-tc-cell"><div class="c3-tc-lbl">${tag}</div><div class="c3-tc-val">${angles[i].toFixed(1)}°</div></div>`).join('') +
      `<div class="c3-tc-cell"><div class="c3-tc-lbl">J5 GRIP</div><div class="c3-tc-val">${Math.round(angles[5])}%</div></div>` +
      `</div><div class="c1-history-empty">TELEMETRY ${hz!=null?('ACTIVE · '+hz.toFixed(1)+' Hz'):'IDLE — no active RTOS cycle'}</div>`;
    return;
  }

  const visible = c3.events.filter(e => _matchesC3EventFilter(e.env)).slice(0, 100);
  if(visible.length === 0){
    list.innerHTML = '<div class="c1-history-empty">No QNX events yet.</div>';
    return;
  }
  list.innerHTML = visible.map(e => {
    const t = new Date(e.ts).toLocaleTimeString('en-GB',{hour12:false});
    return `<div class="c3-event-item${e.id===c3.selectedEventId?' selected':''}" onclick="c3SelectEvent(${e.id})">
      <span class="c3-event-cat cat-${e.env.category}">${e.env.category}</span>
      <span class="c3-event-desc">${escapeHtml(describe(e.env))}</span>
      <span class="c3-event-time">${t}</span>
    </div>`;
  }).join('');
}
function c3SelectEvent(id){
  c3.selectedEventId = id;
  const e = c3.events.find(x => x.id === id);
  if(e) renderInspector(e.env);
  renderC3EventList();
}


// ═══════════════════════════════════════════════════════════════════
//  STATS
// ═══════════════════════════════════════════════════════════════════
function updateStats(env){
  stats.total++;
  stats.byCategory[env.category] = (stats.byCategory[env.category]||0) + 1;
  stats.sizeSum += (env.size_bytes||0);
  const now = Date.now();
  stats.windowTimestamps.push(now);
  const cutoff = now - 3000;
  while(stats.windowTimestamps.length && stats.windowTimestamps[0] < cutoff) stats.windowTimestamps.shift();
}

function updateLatency(env){
  const isCommandish = (env.direction === 'browser_to_bridge') &&
    (env.category === 'command' || env.category === 'sequence' || env.category === 'emergency');
  const isResponse = (env.direction === 'qnx_to_bridge' || env.direction === 'bridge_to_browser') &&
    (env.category === 'telemetry' || env.category === 'ack' || env.category === 'emergency' || env.category === 'system_status');

  if(isCommandish){
    pendingCommand = { desc: describe(env), ts: env.timestamp_ns };
    return;
  }
  if(isResponse && pendingCommand){
    const latencyMs = (env.timestamp_ns - pendingCommand.ts) / 1e6;
    if(latencyMs >= 0 && latencyMs < 10000){
      timelineEntries.unshift({ cmdDesc: pendingCommand.desc, respDesc: describe(env), latencyMs, at: Date.now() });
      if(timelineEntries.length > 30) timelineEntries.pop();
      document.getElementById('stat-latency').textContent = latencyMs.toFixed(1);
      renderTimeline();
    }
    pendingCommand = null;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  DECODING
// ═══════════════════════════════════════════════════════════════════
function describe(env){
  const p = env.payload || {};
  switch(p.type){
    case 'angles': {
      const a = p.angles || [];
      const j = a.map((v,i)=> i===5 ? `Gripper=${Math.round(v)}%` : `J${i}=${v.toFixed(1)}°`).join(' ');
      return `Joint telemetry — ${j} (packet #${p.packets})`;
    }
    case 'status':
      return `QNX ${p.qnx_connected ? 'connected' : 'disconnected'}${p.addr && p.addr!=='—' ? ' — '+p.addr : ''}`;
    case 'browser_status':
      return `Robotic dashboard ${p.browser_connected ? 'connected' : 'disconnected'} (${p.count} client${p.count===1?'':'s'})`;
    case 'emergency':
      return `EMERGENCY STOP — source: ${p.source || 'unknown'}`;
    case 'reset_emergency':
      return `Browser requested EMERGENCY RESET`;
    case 'emergency_cleared':
      return `QNX confirmed recovery — EMERGENCY CLEARED`;
    case 'watchdog':
      return `RTOS watchdog — deadline missed by ${Number(p.missed_ms).toFixed(1)}ms`;
    case 'pose_advance':
      return `QNX advanced to pose #${p.pose_idx}`;
    case 'cycle_done':
      return `RTOS cycle #${p.cycle_num} completed`;
    case 'seq_complete':
      return `RTOS cycling stopped cleanly (SEQ_COMPLETE)`;
    case 'rtos_sequence':
      return `Browser → QNX: ${p.total_poses} poses @ ${p.pose_duration_ms}ms/pose (RTOS)`;
    case 'send_sequence':
      return `Browser → QNX: ${p.total_poses} poses (legacy sequence)`;
    case 'stop_cycle':
      return `Browser requested graceful stop-after-cycle`;
    case 'set_mode':
      return `Browser switched operating mode → ${(p.mode||'').toUpperCase()}`;
    case 'seq_ack':
      return p.ok===false ? `Gateway REJECTED: ${p.msg}` : `Gateway acknowledged: ${p.msg}`;
    case 'fault_config':
      return `Test Lab: fault config ${p.ok ? 'applied' : 'rejected'} — ${p.detail||''}`;
    case 'qnx_tx':
      return `Bridge → QNX (wire): ${(p.lines||[]).join(' / ')}`;
    default:
      if(env.category === 'system_status')
        return `Gateway status snapshot`;
      if(env.category === 'security')
        return `Security alert — ${p.reason || ''} from ${p.ip || '?'}${p.blocked ? ' (IP BLOCKED)' : ''}`;
      return `${env.category} event`;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  DETAIL VIEW — dispatch to the right card-specific layout
// ═══════════════════════════════════════════════════════════════════
const CARD_TITLES = {
  card1: 'ROBOTIC DASHBOARD → PYTHON BRIDGE — Transmission &amp; Robot Investigation',
  card2: 'PYTHON BRIDGE — Communication Journey Investigation',
  card3: 'QNX RTOS — Execution &amp; Output Observatory',
};

function openDetail(cardId){
  currentDetailCard = cardId;
  document.getElementById('cards-view').style.display = 'none';
  document.getElementById('detail-view').style.display = 'flex';
  document.getElementById('detail-title').innerHTML = CARD_TITLES[cardId];

  document.getElementById('c1-detail').style.display = cardId==='card1' ? 'grid' : 'none';
  document.getElementById('c2-detail').style.display = cardId==='card2' ? 'flex' : 'none';
  document.getElementById('c3-detail').style.display = cardId==='card3' ? 'flex' : 'none';

  renderDetailControls(cardId);

  if(cardId === 'card1'){
    initC1Detail();
    renderC1Subfilters();
    renderC1History();
    renderC1Selected();
  } else if(cardId === 'card2'){
    renderC2Subfilters();
    renderC2TxList();
  } else if(cardId === 'card3'){
    renderC3Subfilters();
    renderC3Detail();
    renderC3EventList();
  }
}
function closeDetail(){
  currentDetailCard = null;
  document.getElementById('detail-view').style.display = 'none';
  document.getElementById('cards-view').style.display = 'grid';
}

function renderDetailControls(cardId){
  const wrap = document.getElementById('detail-controls');
  if(!wrap) return;
  wrap.innerHTML = `
    <button id="btn-clear">CLEAR</button>
    <button id="btn-pause">${paused ? '▶ RESUME' : '⏸ PAUSE'}</button>`;
  document.getElementById('btn-clear').onclick = () => {
    if(cardId === 'card1'){ c1.history = []; c1.selectedHistoryId = null; renderC1History(); renderC1Selected(); }
    if(cardId === 'card2'){ packets = []; selectedPacketId = null; renderC2TxList(); document.getElementById('inspector').innerHTML = '<div class="inspector-empty">Select a packet to inspect its envelope.</div>'; }
    if(cardId === 'card3'){ c3.events = []; c3.selectedEventId = null; renderC3EventList(); }
  };
  document.getElementById('btn-pause').onclick = (e) => {
    paused = !paused;
    e.target.textContent = paused ? '▶ RESUME' : '⏸ PAUSE';
    e.target.classList.toggle('active', paused);
    if(!paused){
      if(cardId === 'card2') renderC2TxList();
      if(cardId === 'card3') renderC3EventList();
    }
  };
}

function renderInspector(env){
  const box = document.getElementById('inspector');
  box.innerHTML = `
    <div class="insp-decoded">${escapeHtml(describe(env))}</div>
    <div class="insp-field"><div class="insp-label">PACKET ID</div><div class="insp-val">#${env.packet_id}</div></div>
    <div class="insp-field"><div class="insp-label">DIRECTION</div><div class="insp-val">${env.direction}</div></div>
    <div class="insp-field"><div class="insp-label">CATEGORY</div><div class="insp-val">${env.category}</div></div>
    <div class="insp-field"><div class="insp-label">TRANSPORT</div><div class="insp-val">${env.transport}</div></div>
    <div class="insp-field"><div class="insp-label">SIZE</div><div class="insp-val">${env.size_bytes} bytes</div></div>
    <div class="insp-field"><div class="insp-label">REAL / SIMULATED</div><div class="insp-val" style="color:${env.real?'var(--green)':'var(--amber)'}">${env.real ? 'REAL' : 'SIMULATED'}</div></div>
    <div class="insp-field"><div class="insp-label">REPLAYED</div><div class="insp-val">${env.replayed ? 'Yes — backfilled after reconnect' : 'No — received live'}</div></div>
    <div class="insp-field"><div class="insp-label">GATEWAY TIMESTAMP (monotonic ns)</div><div class="insp-val">${env.timestamp_ns}</div></div>
    <div class="insp-field"><div class="insp-label">RAW ENVELOPE</div><div class="insp-raw">${escapeHtml(JSON.stringify(env, null, 2))}</div></div>
  `;
}

// ═══════════════════════════════════════════════════════════════════
//  RENDERING — Stats
// ═══════════════════════════════════════════════════════════════════
const CATEGORY_COLORS = {
  telemetry:'#00d4ff', command:'#00ff88', ack:'#00ff88', sequence:'#8844ff',
  emergency:'#ff2244', system_status:'#ffaa00', error:'#ff2244', security:'#ff66cc'
};

function renderStats(){
  document.getElementById('stat-total').textContent = stats.total;
  document.getElementById('stat-rate').textContent = Math.round(stats.windowTimestamps.length / 3);
  document.getElementById('stat-size').textContent = stats.total ? Math.round(stats.sizeSum/stats.total) : 0;

  const wrap = document.getElementById('cat-breakdown');
  wrap.innerHTML = '';
  Object.keys(stats.byCategory).sort((a,b)=>stats.byCategory[b]-stats.byCategory[a]).forEach(cat=>{
    const row = document.createElement('div');
    row.className = 'cat-row';
    row.innerHTML = `<span class="cat-swatch" style="background:${CATEGORY_COLORS[cat]||'#888'}"></span>
      <span class="cat-name">${cat}</span><span class="cat-count">${stats.byCategory[cat]}</span>`;
    wrap.appendChild(row);
  });
}

// ═══════════════════════════════════════════════════════════════════
//  RENDERING — Topology
// ═══════════════════════════════════════════════════════════════════
function connState(kind){
  if(kind === 'gw') return topo.gwOnline ? 'on' : (topo.gwConnecting ? 'warn' : 'off');
  if(kind === 'qnx') return topo.qnxOnline===true ? 'on' : topo.qnxOnline===false ? 'off' : 'warn';
  if(kind === 'br') return topo.brOnline ? 'on' : 'off';
  return 'off';
}
const STATE_LABELS = { on: 'ONLINE', warn: 'CONNECTING', off: 'OFFLINE' };

function renderTopology(){
  const nQnx = document.getElementById('node-qnx');
  const nGw  = document.getElementById('node-gw');
  const nBr  = document.getElementById('node-br');

  const sQnx = connState('qnx'), sGw = connState('gw'), sBr = connState('br');
  const now = Date.now();
  const qnxActive = sQnx==='on' && (now - topo.lastQnxTrafficAt) < 2000;
  const gwActive  = sGw==='on'  && (now - topo.lastGwTrafficAt)  < 2000;
  const brActive  = sBr==='on'  && (now - topo.lastBrTrafficAt)  < 2000;

  nQnx.setAttribute('class', 'topo-node ' + sQnx + (qnxActive ? ' active-traffic' : ''));
  nGw.setAttribute('class',  'topo-node ' + sGw  + (gwActive  ? ' active-traffic' : ''));
  nBr.setAttribute('class',  'topo-node ' + sBr  + (brActive  ? ' active-traffic' : ''));

  document.getElementById('topo-state-qnx').textContent = STATE_LABELS[sQnx];
  document.getElementById('topo-state-gw').textContent  = STATE_LABELS[sGw];
  document.getElementById('topo-state-br').textContent  = STATE_LABELS[sBr];
  document.getElementById('topo-activity-qnx').textContent = qnxActive ? 'ACTIVE' : (sQnx==='on' ? 'idle' : '');
  document.getElementById('topo-activity-gw').textContent  = gwActive  ? 'ACTIVE' : (sGw==='on'  ? 'idle' : '');
  document.getElementById('topo-activity-br').textContent  = brActive  ? 'ACTIVE' : (sBr==='on'  ? 'idle' : '');

  document.getElementById('link-qnx-gw').classList.toggle('live', sQnx==='on' && sGw==='on');
  document.getElementById('link-gw-br').classList.toggle('live', sGw==='on' && sBr==='on');
}
setInterval(renderTopology, 800);

function firePulse(hop){
  const el = document.getElementById('pulse-' + hop);
  if(!el) return;
  el.classList.add('active');
  clearTimeout(el._fadeTimer);
  el._fadeTimer = setTimeout(()=>el.classList.remove('active'), 700);
}

// ═══════════════════════════════════════════════════════════════════
//  RENDERING — Timeline
// ═══════════════════════════════════════════════════════════════════
function renderTimeline(){
  const wrap = document.getElementById('timeline-list');
  wrap.innerHTML = '';
  if(timelineEntries.length===0){
    wrap.innerHTML = '<div class="inspector-empty">No paired command→response events yet. Trigger RTOS send, emergency stop, or mode change on the robotic dashboard.</div>';
    return;
  }
  timelineEntries.forEach(e=>{
    const item = document.createElement('div');
    item.className = 'tl-item';
    item.innerHTML = `
      <div class="tl-pair"><span>→ ${escapeHtml(e.cmdDesc)}</span></div>
      <div class="tl-pair"><span>← ${escapeHtml(e.respDesc)}</span><span class="tl-lat">${e.latencyMs.toFixed(1)}ms</span></div>
    `;
    wrap.appendChild(item);
  });
}


// ═══════════════════════════════════════════════════════════════════
//  TEST LAB (Other-Problems §2/§3: redesigned Fault Injection & Experiment
//  Console + compact main-dashboard status widget) — unchanged behavior
// ═══════════════════════════════════════════════════════════════════
const testLab = {
  lastActive: false,
  lastMode: null,
  lastRate: null,
  lastDurationS: null,
  activeStartedAt: null,
  lastKnownRemaining: null,
  history: [],
};
const TESTLAB_HISTORY_MAX = 12;

const FAULT_TYPE_META = {
  packet_loss: { label: 'DROP RATE (%)', toBackendRate: v => Math.min(1, Math.max(0, v/100)), defaultUi: 25 },
  delay:       { label: 'DELAY (ms)',    toBackendRate: v => Math.min(3000, Math.max(0, v))/1000, defaultUi: 400 },
  corrupt:     { label: 'CORRUPT RATE (%)', toBackendRate: v => Math.min(1, Math.max(0, v/100)), defaultUi: 30 },
};

function tlOnFaultTypeChange(){
  const type = document.getElementById('tl-fault-type').value;
  const meta = FAULT_TYPE_META[type];
  const rateLbl = document.getElementById('tl-rate-label');
  const rateInp = document.getElementById('tl-fault-rate');
  if(rateLbl) rateLbl.textContent = meta.label;
  if(rateInp) rateInp.value = meta.defaultUi;
}

function tlArmFault(){
  const type = document.getElementById('tl-fault-type').value;
  const meta = FAULT_TYPE_META[type];
  const uiRate = parseFloat(document.getElementById('tl-fault-rate').value) || meta.defaultUi;
  const duration = Math.max(1, parseInt(document.getElementById('tl-fault-duration').value, 10) || 30);
  const backendRate = meta.toBackendRate(uiRate);
  testLabFault(type, backendRate, duration);
}

function testLabFault(mode, rate, duration_s){
  if(!ws || ws.readyState !== 1 || !authOk){ toast('Not connected/authenticated'); return; }
  ws.send(JSON.stringify({ type: 'test_lab_fault_config', mode, rate, duration_s }));
  toast(`Requested: ${mode} @ rate ${rate} for ${duration_s}s`);
}
function testLabKill(){
  if(!ws || ws.readyState !== 1 || !authOk){ toast('Not connected/authenticated'); return; }
  ws.send(JSON.stringify({ type: 'test_lab_kill' }));
  toast('Kill switch sent — clearing all fault injection');
}

function renderTestLabStatus(fault, ids){
  fault = fault || { active: false };
  const active = !!fault.active;

  if(active && !testLab.lastActive){
    testLab.activeStartedAt = new Date();
    testLab.lastKnownRemaining = fault.remaining_s;
  } else if(!active && testLab.lastActive){
    const wasNearZero = testLab.lastKnownRemaining != null && testLab.lastKnownRemaining <= 1.5;
    testLab.history.unshift({
      mode: testLab.lastMode, rate: testLab.lastRate, durationS: testLab.lastDurationS,
      startedAt: testLab.activeStartedAt, endedAt: new Date(),
      status: wasNearZero ? 'completed' : 'stopped'
    });
    if(testLab.history.length > TESTLAB_HISTORY_MAX) testLab.history.pop();
    testLab.activeStartedAt = null;
  }
  if(active){
    testLab.lastMode = fault.mode;
    testLab.lastRate = fault.rate;
    testLab.lastDurationS = fault.duration_s != null ? fault.duration_s : testLab.lastDurationS;
    testLab.lastKnownRemaining = fault.remaining_s;
  }
  testLab.lastActive = active;

  const widget = document.getElementById('tl-dash-widget');
  const statusText = document.getElementById('tl-dash-status-text');
  const detailEl = document.getElementById('tl-dash-detail');
  if(widget && statusText && detailEl){
    widget.classList.toggle('active', active);
    if(active){
      statusText.textContent = 'FAULT ACTIVE';
      detailEl.innerHTML = `<b>${escapeHtml((fault.mode||'').toUpperCase())}</b> · rate ${escapeHtml(String(fault.rate))} · ${fault.remaining_s!=null ? fault.remaining_s.toFixed(1)+'s remaining' : ''}`;
    } else {
      statusText.textContent = 'INACTIVE';
      detailEl.textContent = 'No fault currently active.';
    }
  }

  const curPanel = document.getElementById('tl-current-panel');
  if(curPanel){
    if(active){
      const pct = (fault.duration_s && fault.remaining_s!=null)
        ? Math.max(0, Math.min(100, 100 * (1 - (fault.remaining_s / fault.duration_s))))
        : null;
      curPanel.innerHTML = `
        <div class="tl-current-type">🔴 FAULT ACTIVE — ${escapeHtml((fault.mode||'').toUpperCase())}</div>
        <div class="tl-current-path">Applied to: Bridge → Browser broadcast (:8765)</div>
        <div class="tl-progress-track"><div class="tl-progress-fill" style="width:${pct!=null?pct:50}%"></div></div>
        <div class="tl-progress-lbl">${fault.remaining_s!=null ? fault.remaining_s.toFixed(1)+'s remaining' : 'remaining time not reported'}</div>
        <button class="tl-kill-btn" onclick="testLabKill()">⛔ KILL / STOP FAULT</button>
      `;
    } else {
      curPanel.innerHTML = `<div class="tl-current-empty">● TEST LAB INACTIVE<br>No fault currently active.</div>`;
    }
  }

  const liveMode = document.getElementById('tl-live-mode');
  const liveRate = document.getElementById('tl-live-rate');
  const liveRemaining = document.getElementById('tl-live-remaining');
  if(liveMode) liveMode.textContent = active ? (fault.mode||'—').toUpperCase() : '—';
  if(liveRate) liveRate.textContent = active ? String(fault.rate) : '—';
  if(liveRemaining) liveRemaining.textContent = active && fault.remaining_s!=null ? fault.remaining_s.toFixed(1) : '—';

  renderTestLabHistory();

  const idsEl = document.getElementById('ids-status');
  const entries = Object.entries(ids || {});
  if(entries.length === 0){
    idsEl.innerHTML = 'No failed-auth activity recorded yet.';
    return;
  }
  idsEl.innerHTML = entries.map(([ip, rec]) => {
    const cls = rec.blocked ? 'ids-row-blocked' : 'ids-row-ok';
    const status = rec.blocked ? `BLOCKED (${rec.blocked_remaining_s}s left)` : `${rec.failures_in_window} failure(s) in window`;
    return `<div class="${cls}">${escapeHtml(ip)} — ${status}</div>`;
  }).join('');
}

function renderTestLabHistory(){
  const wrap = document.getElementById('tl-history-list');
  if(!wrap) return;
  if(testLab.history.length === 0){
    wrap.innerHTML = '<div class="tl-history-empty">No experiments run yet this session.</div>';
    return;
  }
  wrap.innerHTML = testLab.history.map(h => {
    const t = h.startedAt ? h.startedAt.toLocaleTimeString('en-GB',{hour12:false}) : '—';
    return `<div class="tl-history-item">
      <div>
        <div class="tl-history-main">${t} — ${escapeHtml((h.mode||'').toUpperCase())}</div>
        <div class="tl-history-sub">rate ${escapeHtml(String(h.rate))}${h.durationS?(' · '+h.durationS+'s'):''}</div>
      </div>
      <div class="tl-history-status ${h.status}">${h.status === 'completed' ? '✓ COMPLETED' : '✕ STOPPED'}</div>
    </div>`;
  }).join('');
}

document.getElementById('btn-testlab-toggle').addEventListener('click', () => {
  document.getElementById('testlab-overlay').classList.remove('testlab-hidden');
  if(ws && ws.readyState===1 && authOk){
    ws.send(JSON.stringify({ type: 'test_lab_status_query' }));
  }
});
document.getElementById('btn-testlab-close').addEventListener('click', () => {
  document.getElementById('testlab-overlay').classList.add('testlab-hidden');
});

// ═══════════════════════════════════════════════════════════════════
//  UTIL
// ═══════════════════════════════════════════════════════════════════
function escapeHtml(s){
  return String(s).replace(/[&<>"']/g, c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
}

let toastTimer=null;
function toast(msg){
  const el = document.getElementById('toast');
  el.textContent = msg; el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(()=>el.classList.remove('show'), 3200);
}

function log(msg){ console.log('[COMM-DASH] ' + msg); }

// ═══════════════════════════════════════════════════════════════════
//  BOOT
// ═══════════════════════════════════════════════════════════════════
initC1Overview();
renderC1Subfilters();
renderC2Subfilters();
renderC3Subfilters();
renderC3Overview();
renderC3EventList();
renderTopology();
renderStats();
renderTimeline();
connect();
