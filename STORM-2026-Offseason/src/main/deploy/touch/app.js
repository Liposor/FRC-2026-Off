"use strict";

const field = document.getElementById("field");
const context = field.getContext("2d");
const image = new Image();
image.src = "field.png";
image.onload = draw;
const fieldLength = 16.54;
const fieldWidth = 8.07;
const markerColors = {
  edge: "#100b13", point: "#e3b7f7", route: "#ffe1a3", selection: "#ffffff"
};
let snapshot = null;
let chosen = null;
let connected = false;
let busy = false;
let routesKey = "";

const byId = id => document.getElementById(id);
const show = (id, value) => { byId(id).textContent = value; };
const labels = {
  IDLE: "Parado", COLLECTING: "Coletando", HOLDING: "Segurando",
  NAVIGATING: "Indo à zona", AIMING: "Mirando", LOCKING_X: "Travando rodas",
  RECOVERING_RPM: "Recuperando RPM", FEEDING: "Alimentando", EJECTING: "Ejetando",
  CALIBRATION: "Calibração", NAVIGATION: "Em deslocamento", ALLIANCE_UNKNOWN: "Sem aliança",
  POSE_UNREFERENCED: "Referencie a pose", OUTSIDE_ZONE: "Fora da zona",
  AIM: "Ajustando mira", MOVING: "Aguardando parada", WHEEL_LOCK: "Aguardando rodas em X",
  SHOOTER_FEEDBACK: "Verifique shooter e flap", ROLLER_FEEDBACK: "Verifique os rolos",
  POWER_LIMIT: "Limite de tensão", RPM_OR_FLAP: "Aguardando RPM / flap", READY: "Pronto",
  UNKNOWN: "Indeterminado", LIKELY_PRESENT: "Provável presença", LIKELY_EMPTY: "Provável vazio",
  RUNNING: "Em curso", ARRIVED: "Chegou", CANCELLED: "Cancelada", UNAVAILABLE: "Indisponível",
  BLOCKED: "Bloqueada", TIMEOUT: "Tempo esgotado"
};
const shiftNames = {
  AUTO: "Auto", TRANSITION: "Transição", SHIFT_1: "Shift 1", SHIFT_2: "Shift 2",
  SHIFT_3: "Shift 3", SHIFT_4: "Shift 4", ENDGAME: "Endgame"
};
const hubNames = { ACTIVE: "Ativo", INACTIVE: "Inativo", UNKNOWN: "Indefinido" };

function formatTime(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return "—";
  const whole = Math.ceil(seconds);
  return Math.floor(whole / 60) + ":" + String(whole % 60).padStart(2, "0");
}

function renderClock(state) {
  show("match-label", state?.clockSource === "PRACTICE" ? "Partida · treino" : "Partida");
  show("match-time", formatTime(state?.matchTime));
  show("shift-name", shiftNames[state?.shift] || "Shift");
  show("shift-time", formatTime(state?.shiftTime));
  show("hub-status", state?.shift && state.shift !== "NONE" ? (hubNames[state.hub] || "—") : "—");
  byId("hub-clock").dataset.hub = state?.hub || "";
}

const adjustments = [
  ["rpm", "Shooter", " rpm"], ["flap", "Ângulo do flap", "°"],
  ["intake", "Intake", "%"], ["rollers", "Rolos", "%"]
];

for (const [key, title] of adjustments) {
  const card = document.createElement("article");
  card.className = "adjustment";
  const heading = document.createElement("h3");
  heading.textContent = title;
  const controls = document.createElement("div");
  controls.className = "adjustment-controls";
  const value = document.createElement("output");
  value.id = key;
  const minus = document.createElement("button");
  minus.textContent = "−";
  minus.dataset.action = "trim:" + key + ":-1";
  minus.setAttribute("aria-label", "Diminuir " + title);
  const plus = document.createElement("button");
  plus.textContent = "+";
  plus.dataset.action = "trim:" + key + ":1";
  plus.setAttribute("aria-label", "Aumentar " + title);
  controls.append(minus, value, plus);
  card.append(heading, controls);
  byId("adjustments").append(card);
}

function xy(pose) {
  return [pose[0] / fieldLength * field.width, (1 - pose[1] / fieldWidth) * field.height];
}

function marker(pose, color, label) {
  if (!pose) return;
  const [x, y] = xy(pose);
  context.fillStyle = color;
  context.strokeStyle = markerColors.edge;
  context.lineWidth = 4;
  context.beginPath();
  context.arc(x, y, 13, 0, Math.PI * 2);
  context.fill();
  context.stroke();
  if (label) {
    context.fillStyle = markerColors.edge;
    context.font = "bold 16px Segoe UI";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(label, x, y + 1);
  }
}

function draw() {
  context.clearRect(0, 0, field.width, field.height);
  if (image.complete && image.naturalWidth) context.drawImage(image, 0, 0, field.width, field.height);
  if (!snapshot) return;
  if (connected && snapshot.path?.length > 1) {
    context.beginPath();
    context.moveTo(...xy(snapshot.path[0]));
    for (const pose of snapshot.path.slice(1)) context.lineTo(...xy(pose));
    context.lineWidth = 5;
    context.strokeStyle = markerColors.route;
    context.stroke();
  }
  for (const [index, point] of snapshot.points.entries()) marker(point, markerColors.point, index + 1);
  marker(snapshot.target, markerColors.route, "");
  marker(chosen, markerColors.selection, "");
  if (!snapshot.pose || !connected) return;
  const [x, y] = xy(snapshot.pose);
  context.save();
  context.translate(x, y);
  context.rotate(-snapshot.pose[2] * Math.PI / 180);
  context.fillStyle = snapshot.alliance === "Red" ? "#f78484" : "#7dbbff";
  context.strokeStyle = markerColors.edge;
  context.lineWidth = 4;
  context.beginPath();
  context.moveTo(26, 0);
  context.lineTo(-17, 14);
  context.lineTo(-17, -14);
  context.closePath();
  context.fill();
  context.stroke();
  context.restore();
}

function refreshControls() {
  const motion = connected && snapshot?.teleop && snapshot?.remote && !snapshot?.calibration;
  for (const button of document.querySelectorAll("[data-action]")) {
    const needsField = button.hasAttribute("data-field");
    button.disabled = busy || !connected || (button.hasAttribute("data-motion") && !motion)
      || (needsField && (!snapshot?.fieldReady || snapshot?.alliance === "UNKNOWN"));
  }
  byId("go").disabled = busy || !motion || !snapshot?.fieldReady || !chosen || snapshot?.alliance === "UNKNOWN";
  byId("zero").disabled ||= !!snapshot?.enabled;
  byId("calibration-on").disabled ||= !snapshot?.canCalibrate;
  byId("heading-toggle").disabled ||= typeof snapshot?.headingAssist !== "boolean";
  field.classList.toggle("armed", !!(motion && snapshot?.fieldReady));
}

function renderHeading(state) {
  const button = byId("heading-toggle");
  if (typeof state?.headingAssist !== "boolean") {
    show("heading-status", "Sincronizando…");
    return;
  }
  button.dataset.action = state.headingAssist ? "heading-off" : "heading-on";
  button.setAttribute("aria-pressed", String(state.headingAssist));
  const status = {
    OFF: "Desligada", WAITING: state.teleop ? "Aguardando tags" : "Aguardando teleop",
    TRACKING: "Visão confiável", HOLDING: "Mantendo correção"
  }[state.headingAssistStatus] || "Sincronizando…";
  const offset = state.headingAssistOffset;
  const degrees = Number.isFinite(offset) && Math.abs(offset) >= 0.05
    ? " · " + (offset > 0 ? "+" : "") + offset.toFixed(1) + "°" : "";
  show("heading-status", status + degrees);
}

function render(latency) {
  const state = snapshot;
  renderClock(state);
  renderHeading(state);
  show("battery", state.battery.toFixed(1) + " V");
  show("ping", Math.round(latency) + " ms");
  show("alliance", { Red: "Vermelha", Blue: "Azul" }[state.alliance] || "—");
  show("reference", state.fieldReady ? "Pose referenciada" : "Pose sem referência");
  byId("reference").classList.toggle("ready", state.fieldReady);
  show("connection", "Conectado");
  byId("dot").classList.add("online");
  byId("match").hidden = state.calibration;
  byId("calibration").hidden = !state.calibration;
  show("phase", labels[state.phase] || state.phase);
  show("gate", labels[state.gate] || state.gate);
  show("fuel", labels[state.fuel] || state.fuel);
  show("nav", labels[state.nav] || state.nav);
  show("result", state.result);
  const pose = state.pose;
  show("pose", pose ? "X " + pose[0].toFixed(2) + "  Y " + pose[1].toFixed(2)
    + "  θ " + pose[2].toFixed(0) + "°" : "Pose indisponível");
  show("permit", state.remote ? "LB ativo · toque no mapa para escolher destino."
    : "Segure LB no controle 2 para operar pela tela.");
  show("spin", state.spin ? "Desarmar teste do shooter" : "Armar teste do shooter");
  for (const [key, , unit] of adjustments) {
    show(key, (state[key] > 0 && (key === "rpm" || key === "flap") ? "+" : "") + state[key] + unit);
  }
  const newRoutes = state.routes.join("|");
  if (routesKey !== newRoutes) {
    routesKey = newRoutes;
    const buttons = state.routes.map(route => {
      const button = document.createElement("button");
      button.textContent = route.replace("Coleta ", "");
      button.dataset.action = "route:" + route;
      button.dataset.motion = "";
      button.dataset.field = "";
      return button;
    });
    byId("routes").replaceChildren(...buttons);
  }
  refreshControls();
  draw();
}

async function poll() {
  const start = performance.now();
  try {
    const response = await fetch("api/status", {cache: "no-store", signal: AbortSignal.timeout(900)});
    if (!response.ok) throw Error("HTTP " + response.status);
    const next = await response.json();
    if (!next.token) throw Error("Inicializando");
    if (snapshot && (snapshot.alliance !== next.alliance || snapshot.token !== next.token)) chosen = null;
    snapshot = next;
    connected = true;
    render(performance.now() - start);
  } catch {
    connected = false;
    show("connection", "Sem conexão");
    show("ping", "—");
    renderClock(null);
    renderHeading(null);
    show("result", "Reconectando ao roboRIO…");
    byId("dot").classList.remove("online");
    refreshControls();
    draw();
  } finally {
    setTimeout(poll, 200);
  }
}

async function send(action) {
  if (!connected || busy || !snapshot) return;
  busy = true;
  refreshControls();
  try {
    const result = await fetch("api/action", {
      method: "POST", headers: {"X-Storm-Session": snapshot.token}, body: action,
      signal: AbortSignal.timeout(900)
    });
    if (!result.ok) throw Error("HTTP " + result.status);
    show("result", "Aguardando resposta do robô…");
  } catch (error) {
    show("result", "Falha ao enviar · " + error.message);
  } finally {
    busy = false;
    refreshControls();
  }
}

document.addEventListener("click", event => {
  const button = event.target.closest("[data-action]");
  if (button && !button.disabled) send(button.dataset.action);
});

field.addEventListener("click", event => {
  if (!field.classList.contains("armed")) return;
  const rect = field.getBoundingClientRect();
  chosen = [(event.clientX - rect.left) / rect.width * fieldLength,
    (1 - (event.clientY - rect.top) / rect.height) * fieldWidth];
  draw();
  refreshControls();
});

byId("go").addEventListener("click", () => {
  if (chosen) send("goto:" + chosen.map(value => value.toFixed(3)).join(":"));
});

refreshControls();
poll();

