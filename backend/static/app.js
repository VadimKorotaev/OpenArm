const state = {
  health: null,
  skills: [],
  selected: null,
  poses: { left: null, right: null },
  markers: [],
  execution: null,
  query: "",
  dirty: false,
};

const dialogState = { index: null, draft: null };
const movePoseRequests = new WeakMap();
const movePoseSelections = new WeakMap();

const dom = {
  system: document.querySelector("#system-state"),
  skillsCount: document.querySelector("#skills-count"),
  skillList: document.querySelector("#skill-list"),
  search: document.querySelector("#skill-search"),
  name: document.querySelector("#skill-name"),
  description: document.querySelector("#skill-description"),
  actionList: document.querySelector("#action-list"),
  actionCount: document.querySelector("#action-count"),
  save: document.querySelector("#save-skill-button"),
  run: document.querySelector("#run-skill-button"),
  remove: document.querySelector("#delete-skill-button"),
  cancel: document.querySelector("#cancel-button"),
  execution: document.querySelector("#execution-state"),
  leftPose: document.querySelector("#left-arm-readout .pose-value"),
  rightPose: document.querySelector("#right-arm-readout .pose-value"),
  nudgeArm: document.querySelector("#nudge-arm"),
  nudgeStep: document.querySelector("#nudge-step"),
  nudgeAngleStep: document.querySelector("#nudge-angle-step"),
  captureReference: document.querySelector("#capture-reference"),
  captureFrame: document.querySelector("#capture-frame"),
  capturePose: document.querySelector("#capture-pose-button"),
  markerCount: document.querySelector("#marker-count"),
  markerSummary: document.querySelector("#marker-summary"),
  markerFrame: document.querySelector("#marker-frame"),
  markerX: document.querySelector("#marker-x"),
  markerY: document.querySelector("#marker-y"),
  markerZ: document.querySelector("#marker-z"),
  markerRoll: document.querySelector("#marker-roll"),
  markerPitch: document.querySelector("#marker-pitch"),
  markerYaw: document.querySelector("#marker-yaw"),
  dialog: document.querySelector("#action-dialog"),
  dialogTitle: document.querySelector("#action-dialog-title"),
  dialogBody: document.querySelector("#action-dialog-body"),
  actionForm: document.querySelector("#action-form"),
  toast: document.querySelector("#toast"),
};

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
  });
  if (!response.ok) {
    let detail = `${response.status} ${response.statusText}`;
    try {
      const body = await response.json();
      detail = typeof body.detail === "string" ? body.detail : JSON.stringify(body.detail);
    } catch (_) {}
    throw new Error(detail);
  }
  if (response.status === 204) return null;
  return response.json();
}

function toast(message, error = false) {
  dom.toast.textContent = message;
  dom.toast.className = `toast visible${error ? " error" : ""}`;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => (dom.toast.className = "toast"), 3200);
}

function setSystemStatus(ok, label) {
  dom.system.innerHTML = "";
  const dot = document.createElement("span");
  dot.className = `status-dot ${ok ? "online" : "error"}`;
  const labelNode = document.createElement("span");
  labelNode.textContent = label;
  dom.system.append(dot, labelNode);
}

function newId() {
  return crypto.randomUUID();
}

function currentWorldPose(arm) {
  return structuredClone(state.poses[arm]?.pose || {
    position: { x: 0.30, y: arm === "left" ? 0.20 : -0.20, z: 0.40 },
    orientation: { x: 0, y: 0, z: 0, w: 1 },
  });
}

function defaultAction(type, arm = "left") {
  if (type === "move") {
    return {
      action_id: newId(), type, arm,
      target: {
        reference: { kind: "world", frame_id: "world" },
        pose: currentWorldPose(arm),
      },
      velocity_scale: 0.10, acceleration_scale: 0.10, planning_time: 5,
      position_tolerance: 0.002, orientation_tolerance: 0.01,
    };
  }
  if (type === "gripper") {
    return { action_id: newId(), type, arm, opening: 1, max_effort: 0, timeout: 5 };
  }
  if (type === "wait") return { action_id: newId(), type, duration: 1 };
  return {
    action_id: newId(), type: "parallel",
    branches: [
      { actions: [defaultAction("move", "left")] },
      { actions: [defaultAction("move", "right")] },
    ],
  };
}

function actionLabel(action) {
  if (action.type === "move") {
    const p = action.target.pose.position;
    const ref = action.target.reference;
    return `${action.arm.toUpperCase()} · ${ref.kind}:${ref.frame_id} · xyz ${p.x.toFixed(3)} ${p.y.toFixed(3)} ${p.z.toFixed(3)}`;
  }
  if (action.type === "gripper") return `${action.arm.toUpperCase()} · opening ${(action.opening * 100).toFixed(0)}%`;
  if (action.type === "wait") return `${action.duration.toFixed(2)} s`;
  if (action.type === "parallel") {
    const sizes = action.branches.map((branch) => branch.actions.length).join(" + ");
    return `${action.branches.length} ветви · ${sizes} действий`;
  }
  return action.type;
}

function renderSkills() {
  const query = state.query.trim().toLowerCase();
  const items = state.skills.filter((skill) => `${skill.name} ${skill.description}`.toLowerCase().includes(query));
  dom.skillsCount.textContent = state.skills.length;
  dom.skillList.innerHTML = "";
  if (!items.length) {
    const empty = document.createElement("div");
    empty.className = "empty-small";
    empty.textContent = state.skills.length ? "Ничего не найдено" : "Пока нет Skills";
    dom.skillList.append(empty);
    return;
  }
  for (const skill of items) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `skill-item${state.selected?.id === skill.id ? " active" : ""}`;
    const name = document.createElement("span");
    name.className = "skill-name";
    name.textContent = skill.name;
    const meta = document.createElement("span");
    meta.className = "skill-meta";
    const steps = document.createElement("span");
    steps.textContent = `${skill.actions.length} шаг.`;
    const version = document.createElement("span");
    version.textContent = `v${skill.schema_version}`;
    meta.append(steps, version);
    button.append(name, meta);
    button.addEventListener("click", () => selectSkill(skill.id));
    dom.skillList.append(button);
  }
}

function moveAction(index, offset) {
  const target = index + offset;
  if (!state.selected || target < 0 || target >= state.selected.actions.length) return;
  const actions = state.selected.actions;
  [actions[index], actions[target]] = [actions[target], actions[index]];
  state.dirty = true;
  renderActions();
}

function removeAction(index) {
  if (!state.selected) return;
  if (state.selected.actions.length === 1) {
    toast("В Skill должно остаться хотя бы одно действие", true);
    return;
  }
  state.selected.actions.splice(index, 1);
  state.dirty = true;
  renderActions();
}

function actionButton(label, className, callback) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = className;
  button.textContent = label;
  button.addEventListener("click", callback);
  return button;
}

function renderActions() {
  const actions = state.selected?.actions || [];
  dom.actionCount.textContent = `${actions.length} ${actions.length === 1 ? "действие" : "действий"}${state.dirty ? " · не сохранено" : ""}`;
  dom.actionList.innerHTML = "";
  if (!state.selected) {
    dom.actionList.innerHTML = `<div class="empty-state" aria-hidden="true"></div>`;
    return;
  }
  actions.forEach((action, index) => {
    const card = document.createElement("article");
    card.className = "action-card";
    const step = document.createElement("span");
    step.className = "step-index";
    step.textContent = String(index + 1).padStart(2, "0");
    const content = document.createElement("div");
    const title = document.createElement("div");
    title.className = "action-title";
    const glyph = document.createElement("i");
    glyph.className = `action-glyph ${action.type === "gripper" ? "grip" : action.type}`;
    glyph.textContent = action.type[0].toUpperCase();
    const label = document.createElement("span");
    label.textContent = action.type[0].toUpperCase() + action.type.slice(1);
    const tag = document.createElement("span");
    tag.className = "action-tag";
    tag.textContent = action.arm || "flow";
    title.append(glyph, label, tag);
    const summary = document.createElement("div");
    summary.className = "action-summary";
    summary.textContent = actionLabel(action);
    content.append(title, summary);
    const controls = document.createElement("div");
    controls.className = "action-card-controls";
    const edit = actionButton("Edit", "edit-action", () => openActionDialog(action.type, index));
    const up = actionButton("↑", "", () => moveAction(index, -1));
    const down = actionButton("↓", "", () => moveAction(index, 1));
    const remove = actionButton("×", "", () => removeAction(index));
    up.disabled = index === 0;
    down.disabled = index === actions.length - 1;
    controls.append(edit, up, down, remove);
    card.append(step, content, controls);
    dom.actionList.append(card);
  });
}

function setEditorEnabled(enabled) {
  [dom.name, dom.description, dom.save, dom.run, dom.remove].forEach((el) => (el.disabled = !enabled));
  document.querySelectorAll("[data-add-action]").forEach((el) => (el.disabled = !enabled));
  dom.capturePose.disabled = !enabled;
}

function selectSkill(id) {
  if (state.dirty && state.selected?.id !== id && !confirm("Отменить несохранённые изменения?")) return;
  const source = state.skills.find((skill) => skill.id === id);
  state.selected = source ? structuredClone(source) : null;
  state.dirty = false;
  if (state.selected) {
    dom.name.value = state.selected.name;
    dom.description.value = state.selected.description || "";
    setEditorEnabled(true);
  }
  renderSkills();
  renderActions();
}

function poseText(target) {
  if (!target) return "TF недоступен";
  const { position: p, orientation: q } = target.pose;
  const rpy = eulerDegreesFromQuaternion(q);
  return `x ${p.x.toFixed(3)}   y ${p.y.toFixed(3)}   z ${p.z.toFixed(3)}\nrpy° ${rpy.roll.toFixed(1)}  ${rpy.pitch.toFixed(1)}  ${rpy.yaw.toFixed(1)}`;
}

function renderTelemetry() {
  dom.leftPose.textContent = poseText(state.poses.left);
  dom.rightPose.textContent = poseText(state.poses.right);
  dom.markerCount.textContent = state.markers.length;
  if (state.markers.length) {
    dom.markerSummary.textContent = state.markers.map((marker) => {
      const p = marker.pose.position;
      return `${marker.frame_id}\nx ${p.x.toFixed(3)}  y ${p.y.toFixed(3)}  z ${p.z.toFixed(3)}`;
    }).join("\n\n");
  } else {
    dom.markerSummary.textContent = "Маркер не опубликован";
  }
}

function renderExecution() {
  const execution = state.execution;
  const active = Boolean(execution && ["queued", "running"].includes(execution.status));
  document.querySelectorAll("[data-nudge-axis], [data-nudge-rotation-axis]").forEach((button) => {
    button.disabled = active || !state.health?.ros_ready;
  });
  if (!execution) {
    dom.execution.className = "execution-empty";
    dom.execution.innerHTML = `<span class="pulse-ring"></span><div><strong>Нет активного запуска</strong><small>Выберите Skill и нажмите «Запустить»</small></div>`;
    dom.cancel.disabled = true;
    return;
  }
  dom.execution.className = `execution-empty ${execution.status}`;
  dom.execution.innerHTML = "";
  const ring = document.createElement("span");
  ring.className = "pulse-ring";
  const copy = document.createElement("div");
  const title = document.createElement("strong");
  title.textContent = `${execution.skill_name} · ${execution.status}`;
  const detail = document.createElement("small");
  detail.textContent = execution.error || execution.current_path || "Подготовка";
  copy.append(title, detail);
  dom.execution.append(ring, copy);
  dom.cancel.disabled = !active;
}

async function refresh({ quiet = false } = {}) {
  try {
    const [health, skills, markers, left, right, executions] = await Promise.all([
      api("/health"), api("/api/skills"), api("/api/markers"),
      api("/api/robot/arms/left/pose"), api("/api/robot/arms/right/pose"), api("/api/executions"),
    ]);
    state.health = health;
    state.skills = skills;
    state.markers = markers;
    state.poses = { left, right };
    const active = executions.find((item) => ["queued", "running"].includes(item.status));
    state.execution = active || executions[0] || state.execution;
    setSystemStatus(health.ros_ready, health.ros_ready ? "ROS готов" : "ROS недоступен");
    if (state.selected && !state.dirty) {
      const updated = skills.find((skill) => skill.id === state.selected.id);
      if (updated && document.activeElement !== dom.name && document.activeElement !== dom.description) {
        state.selected = structuredClone(updated);
      }
    } else if (!state.selected && skills.length) {
      state.selected = structuredClone(skills[0]);
      dom.name.value = state.selected.name;
      dom.description.value = state.selected.description || "";
      setEditorEnabled(true);
    }
    renderSkills(); renderActions(); renderTelemetry(); renderExecution();
  } catch (error) {
    setSystemStatus(false, "Нет связи");
    if (!quiet) toast(error.message, true);
  }
}

async function saveSkill() {
  if (!state.selected) return false;
  try {
    const updated = await api(`/api/skills/${state.selected.id}`, {
      method: "PATCH",
      body: JSON.stringify({
        name: dom.name.value.trim(),
        description: dom.description.value.trim(),
        actions: state.selected.actions,
      }),
    });
    state.selected = structuredClone(updated);
    state.dirty = false;
    toast("Skill сохранён");
    await refresh({ quiet: true });
    return true;
  } catch (error) {
    toast(error.message, true);
    return false;
  }
}

async function runSkill() {
  if (!state.selected) return;
  if (state.dirty && !(await saveSkill())) return;
  try {
    state.execution = await api(`/api/skills/${state.selected.id}/executions`, { method: "POST" });
    renderExecution();
    toast("Skill запущен");
  } catch (error) { toast(error.message, true); }
}

async function newSkill() {
  if (state.dirty && !confirm("Создать новый Skill и отменить несохранённые изменения?")) return;
  try {
    const created = await api("/api/skills", {
      method: "POST",
      body: JSON.stringify({ name: "Новый Skill", description: "", actions: [defaultAction("wait")] }),
    });
    await refresh({ quiet: true });
    selectSkill(created.id);
    dom.name.focus(); dom.name.select();
    toast("Создан новый Skill");
  } catch (error) { toast(error.message, true); }
}

async function deleteSkill() {
  if (!state.selected || !confirm(`Удалить «${state.selected.name}»?`)) return;
  try {
    await api(`/api/skills/${state.selected.id}`, { method: "DELETE" });
    state.selected = null;
    state.dirty = false;
    dom.name.value = "Выберите Skill"; dom.description.value = "";
    setEditorEnabled(false);
    await refresh({ quiet: true });
    toast("Skill удалён");
  } catch (error) { toast(error.message, true); }
}

async function cancelExecution() {
  if (!state.execution) return;
  try {
    await api(`/api/executions/${state.execution.id}/cancel`, { method: "POST" });
    toast("Запрошена остановка");
    await refresh({ quiet: true });
  } catch (error) { toast(error.message, true); }
}

async function nudgeArm(mode, axis, sign) {
  const arm = dom.nudgeArm.value;
  const translation = { x: 0, y: 0, z: 0 };
  const rotationRpy = { x: 0, y: 0, z: 0 };
  const isRotation = mode === "rotation";
  const step = Number(isRotation ? dom.nudgeAngleStep.value : dom.nudgeStep.value) * sign;
  if (isRotation) rotationRpy[axis] = step * Math.PI / 180;
  else translation[axis] = step;
  try {
    state.execution = await api(`/api/robot/arms/${arm}/nudge`, {
      method: "POST",
      body: JSON.stringify({ translation, rotation_rpy: rotationRpy }),
    });
    renderExecution();
    const axisLabel = isRotation ? ({ x: "Roll", y: "Pitch", z: "Yaw" })[axis] : axis.toUpperCase();
    const valueLabel = isRotation ? `${Math.abs(step).toFixed(0)}°` : `${(Math.abs(step) * 1000).toFixed(0)} mm`;
    toast(`${arm.toUpperCase()}: ${sign > 0 ? "+" : "−"}${axisLabel} ${valueLabel}`);
  } catch (error) { toast(error.message, true); }
}

async function capturePose() {
  if (!state.selected) return;
  if (state.dirty && !(await saveSkill())) return;
  const kind = dom.captureReference.value;
  const frameId = kind === "world" ? "world" : dom.captureFrame.value.trim();
  try {
    const updated = await api(`/api/skills/${state.selected.id}/actions/capture-pose?arm=${dom.nudgeArm.value}`, {
      method: "POST",
      body: JSON.stringify({ reference: { kind, frame_id: frameId } }),
    });
    state.selected = structuredClone(updated);
    state.dirty = false;
    state.skills = state.skills.map((skill) => skill.id === updated.id ? updated : skill);
    renderSkills(); renderActions();
    toast(`Pose ${dom.nudgeArm.value.toUpperCase()} сохранена относительно ${frameId}`);
  } catch (error) { toast(error.message, true); }
}

function quaternionFromEulerDegrees(rollDegrees, pitchDegrees, yawDegrees) {
  const roll = rollDegrees * Math.PI / 180;
  const pitch = pitchDegrees * Math.PI / 180;
  const yaw = yawDegrees * Math.PI / 180;
  const cr = Math.cos(roll / 2);
  const sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2);
  const sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2);
  const sy = Math.sin(yaw / 2);

  return {
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
    w: cr * cp * cy + sr * sp * sy,
  };
}

function eulerDegreesFromQuaternion(quaternion) {
  const norm = Math.hypot(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  const q = norm > 0 ? {
    x: quaternion.x / norm,
    y: quaternion.y / norm,
    z: quaternion.z / norm,
    w: quaternion.w / norm,
  } : { x: 0, y: 0, z: 0, w: 1 };
  const sinPitch = Math.max(-1, Math.min(1, 2 * (q.w * q.y - q.z * q.x)));

  return {
    roll: Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / Math.PI,
    pitch: Math.asin(sinPitch) * 180 / Math.PI,
    yaw: Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / Math.PI,
  };
}

function markerPayload() {
  const roll = Number(dom.markerRoll.value);
  const pitch = Number(dom.markerPitch.value);
  const yaw = Number(dom.markerYaw.value);
  if (![roll, pitch, yaw].every(Number.isFinite)) {
    throw new Error("Roll, Pitch и Yaw должны быть числами");
  }

  return {
    position: { x: Number(dom.markerX.value), y: Number(dom.markerY.value), z: Number(dom.markerZ.value) },
    orientation: quaternionFromEulerDegrees(roll, pitch, yaw),
  };
}

async function setMarker() {
  const frameId = dom.markerFrame.value.trim();
  try {
    await api(`/api/markers/${encodeURIComponent(frameId)}`, { method: "PUT", body: JSON.stringify(markerPayload()) });
    toast(`TF ${frameId} опубликован`);
    await refresh({ quiet: true });
  } catch (error) { toast(error.message, true); }
}

async function deleteMarker() {
  const frameId = dom.markerFrame.value.trim();
  try {
    await api(`/api/markers/${encodeURIComponent(frameId)}`, { method: "DELETE" });
    toast(`TF ${frameId} удалён`);
    await refresh({ quiet: true });
  } catch (error) { toast(error.message, true); }
}

function field(labelText, input) {
  const label = document.createElement("label");
  label.className = "field";
  const title = document.createElement("span");
  title.textContent = labelText;
  label.append(title, input);
  return label;
}

function numberInput(value, step, onChange, min = null, max = null) {
  const input = document.createElement("input");
  input.type = "number";
  // No min/max attributes: with a min set, the browser measures `step` from it
  // and rejects defaults such as timeout 5 s (step 0.5 from 0.1), which would
  // block the whole dialog. The range is enforced on commit instead.
  input.step = String(step);
  input.value = String(value);
  const clamp = (number) => {
    if (min !== null && number < min) return min;
    if (max !== null && number > max) return max;
    return number;
  };
  input.addEventListener("input", () => {
    if (Number.isFinite(input.valueAsNumber)) onChange(clamp(input.valueAsNumber));
  });
  input.addEventListener("change", () => {
    if (!Number.isFinite(input.valueAsNumber)) return;
    const clamped = clamp(input.valueAsNumber);
    if (clamped !== input.valueAsNumber) input.value = String(clamped);
    onChange(clamped);
  });
  return input;
}

function selectInput(value, options, onChange) {
  const select = document.createElement("select");
  for (const [optionValue, label] of options) {
    const option = document.createElement("option");
    option.value = optionValue;
    option.textContent = label;
    option.selected = optionValue === value;
    select.append(option);
  }
  select.addEventListener("change", () => onChange(select.value));
  return select;
}

function textInput(value, onChange) {
  const input = document.createElement("input");
  input.value = value;
  input.addEventListener("input", () => onChange(input.value.trim()));
  return input;
}

function actionIsInCurrentDialog(action) {
  if (dialogState.draft === action) return true;
  if (dialogState.draft?.type !== "parallel") return false;
  return dialogState.draft.branches.some((branch) => branch.actions.includes(action));
}

async function loadCurrentMovePose(action, changes) {
  const currentSelection = movePoseSelections.get(action) || {
    arm: action.arm,
    kind: action.target.reference.kind,
    frameId: action.target.reference.frame_id,
  };
  const selection = { ...currentSelection, ...changes };
  movePoseSelections.set(action, selection);
  const requestId = (movePoseRequests.get(action) || 0) + 1;
  movePoseRequests.set(action, requestId);
  const query = new URLSearchParams({
    reference_kind: selection.kind,
    frame_id: selection.frameId,
  });
  try {
    const target = await api(`/api/robot/arms/${encodeURIComponent(selection.arm)}/pose?${query}`);
    if (movePoseRequests.get(action) !== requestId) return;
    movePoseSelections.delete(action);
    action.arm = selection.arm;
    action.target = target;
    if (actionIsInCurrentDialog(action)) renderActionDialogBody();
  } catch (error) {
    if (movePoseRequests.get(action) !== requestId) return;
    movePoseSelections.delete(action);
    if (actionIsInCurrentDialog(action)) renderActionDialogBody();
    toast(`Не удалось получить текущую pose: ${error.message}`, true);
  }
}

function grid(...nodes) {
  const wrapper = document.createElement("div");
  wrapper.className = "field-grid";
  wrapper.append(...nodes);
  return wrapper;
}

function section(title, ...nodes) {
  const wrapper = document.createElement("section");
  wrapper.className = "dialog-section";
  const heading = document.createElement("div");
  heading.className = "dialog-section-title";
  heading.textContent = title;
  wrapper.append(heading, ...nodes);
  return wrapper;
}

function renderSimpleFields(container, action) {
  if (action.type === "move") {
    const arm = selectInput(action.arm, [["left", "Левая"], ["right", "Правая"]], (value) => {
      loadCurrentMovePose(action, { arm: value });
    });
    const kind = selectInput(action.target.reference.kind, [["world", "World"], ["marker", "Marker TF"]], (value) => {
      loadCurrentMovePose(action, {
        kind: value,
        frameId: value === "world" ? "world" : (state.markers[0]?.frame_id || "aruco_marker_1"),
      });
    });
    const frame = textInput(action.target.reference.frame_id, () => {});
    frame.addEventListener("change", () => {
      loadCurrentMovePose(action, {
        frameId: frame.value.trim(),
      });
    });
    frame.disabled = action.target.reference.kind === "world";
    container.append(section("Рука и система координат", grid(field("Arm", arm), field("Reference", kind)), field("Frame ID", frame)));

    const p = action.target.pose.position;
    const q = action.target.pose.orientation;
    const rpy = eulerDegreesFromQuaternion(q);
    const updateOrientation = () => Object.assign(
      q,
      quaternionFromEulerDegrees(rpy.roll, rpy.pitch, rpy.yaw),
    );
    container.append(section("Позиция, m", grid(
      field("X", numberInput(p.x, 0.001, (v) => { p.x = v; })),
      field("Y", numberInput(p.y, 0.001, (v) => { p.y = v; })),
      field("Z", numberInput(p.z, 0.001, (v) => { p.z = v; })),
    )));
    container.append(section("Ориентация RPY, °", grid(
      field("Roll", numberInput(Number(rpy.roll.toFixed(3)), 1, (v) => { rpy.roll = v; updateOrientation(); })),
      field("Pitch", numberInput(Number(rpy.pitch.toFixed(3)), 1, (v) => { rpy.pitch = v; updateOrientation(); })),
      field("Yaw", numberInput(Number(rpy.yaw.toFixed(3)), 1, (v) => { rpy.yaw = v; updateOrientation(); })),
    )));
    container.append(section("Планирование", grid(
      field("Velocity", numberInput(action.velocity_scale, 0.01, (v) => { action.velocity_scale = v; }, 0.01, 1)),
      field("Acceleration", numberInput(action.acceleration_scale, 0.01, (v) => { action.acceleration_scale = v; }, 0.01, 1)),
      field("Planning time, s", numberInput(action.planning_time, 0.5, (v) => { action.planning_time = v; }, 0.1, 60)),
      field("Position tol., m", numberInput(action.position_tolerance, 0.001, (v) => { action.position_tolerance = v; }, 0.0001, 0.05)),
      field("Orientation tol., rad", numberInput(action.orientation_tolerance, 0.001, (v) => { action.orientation_tolerance = v; }, 0.001, 0.5)),
    )));
    return;
  }
  if (action.type === "gripper") {
    container.append(section("Команда gripper", grid(
      field("Arm", selectInput(action.arm, [["left", "Левая"], ["right", "Правая"]], (v) => { action.arm = v; })),
      field("Opening 0…1", numberInput(action.opening, 0.05, (v) => { action.opening = v; }, 0, 1)),
      field("Max effort", numberInput(action.max_effort, 0.1, (v) => { action.max_effort = v; }, 0)),
      field("Timeout, s", numberInput(action.timeout, 0.5, (v) => { action.timeout = v; }, 0.1, 60)),
    )));
    return;
  }
  if (action.type === "wait") {
    container.append(section("Пауза", field("Duration, s", numberInput(action.duration, 0.1, (v) => { action.duration = v; }, 0.01, 3600))));
  }
}

function renderParallel(container, action) {
  const note = document.createElement("p");
  note.className = "dialog-note";
  note.textContent = "Ветви выполняются одновременно. Одна рука не может использоваться в двух ветвях.";
  container.append(note);
  action.branches.forEach((branch, branchIndex) => {
    const branchNode = document.createElement("section");
    branchNode.className = "parallel-branch";
    const head = document.createElement("div");
    head.className = "branch-head";
    const title = document.createElement("strong");
    title.textContent = `Ветвь ${branchIndex + 1}`;
    head.append(title);
    const actionsNode = document.createElement("div");
    actionsNode.className = "branch-actions";
    branch.actions.forEach((child, childIndex) => {
      const childNode = document.createElement("div");
      childNode.className = "branch-child";
      const copy = document.createElement("div");
      copy.className = "branch-child-copy";
      const name = document.createElement("strong");
      name.textContent = `${childIndex + 1}. ${child.type}`;
      const summary = document.createElement("small");
      summary.textContent = actionLabel(child);
      copy.append(name, summary);
      const controls = document.createElement("div");
      controls.className = "branch-child-controls";
      controls.append(
        actionButton("Edit", "", () => editParallelChild(branchIndex, childIndex)),
        actionButton("×", "", () => {
          if (branch.actions.length === 1) return toast("В ветви должно быть действие", true);
          branch.actions.splice(childIndex, 1); renderActionDialogBody();
        }),
      );
      childNode.append(copy, controls);
      actionsNode.append(childNode);
    });
    const add = document.createElement("div");
    add.className = "branch-add";
    const addLabel = document.createElement("span");
    addLabel.textContent = "Добавить:";
    add.append(addLabel);
    for (const type of ["move", "gripper", "wait"]) {
      add.append(actionButton(type, "", () => {
        branch.actions.push(defaultAction(type, branchIndex === 0 ? "left" : "right"));
        renderActionDialogBody();
      }));
    }
    branchNode.append(head, actionsNode, add);
    container.append(branchNode);
  });
}

function editParallelChild(branchIndex, childIndex) {
  const child = dialogState.draft.branches[branchIndex].actions[childIndex];
  dom.dialogBody.innerHTML = "";
  const back = actionButton("← Назад к ветвям", "ghost-button", renderActionDialogBody);
  back.style.marginBottom = "12px";
  dom.dialogBody.append(back);
  renderSimpleFields(dom.dialogBody, child);
}

function renderActionDialogBody() {
  dom.dialogBody.innerHTML = "";
  if (dialogState.draft.type === "parallel") renderParallel(dom.dialogBody, dialogState.draft);
  else renderSimpleFields(dom.dialogBody, dialogState.draft);
}

function openActionDialog(type, index = null) {
  dialogState.index = index;
  dialogState.draft = index === null ? defaultAction(type) : structuredClone(state.selected.actions[index]);
  dom.dialogTitle.textContent = `${index === null ? "Добавить" : "Изменить"} ${type}`;
  renderActionDialogBody();
  dom.dialog.showModal();
}

function commitAction() {
  if (!state.selected || !dialogState.draft) return;
  if (dialogState.index === null) state.selected.actions.push(dialogState.draft);
  else state.selected.actions[dialogState.index] = dialogState.draft;
  state.dirty = true;
  renderActions();
}

function registerWebMCPTools() {
  const context = document.modelContext;
  if (!context?.registerTool) return;
  const register = (tool) => {
    try { void Promise.resolve(context.registerTool(tool)).catch(() => {}); } catch (_) {}
  };
  register({
    name: "read_openarm_status", title: "Read OpenArm status",
    description: "Read ROS readiness, live arm poses, published marker IDs, and current execution.",
    inputSchema: { type: "object", properties: {}, additionalProperties: false },
    annotations: { readOnlyHint: true, untrustedContentHint: false },
    async execute() {
      await refresh({ quiet: true });
      return { health: state.health, poses: state.poses, markers: state.markers.map((m) => m.frame_id), execution: state.execution };
    },
  });
  register({
    name: "start_skill_execution", title: "Start saved Skill",
    description: "Start one already-saved OpenArm Skill by its exact ID and update the visible execution panel.",
    inputSchema: { type: "object", properties: { skillId: { type: "string" } }, required: ["skillId"], additionalProperties: false },
    annotations: { readOnlyHint: false, untrustedContentHint: false },
    async execute(input) {
      if (!state.skills.some((skill) => skill.id === input.skillId)) throw new Error("unknown skillId");
      state.execution = await api(`/api/skills/${input.skillId}/executions`, { method: "POST" });
      renderExecution();
      return { executionId: state.execution.id, status: state.execution.status };
    },
  });
  register({
    name: "nudge_openarm", title: "Nudge OpenArm end effector",
    description: "Move one end effector along one world axis by a small signed distance in millimeters.",
    inputSchema: {
      type: "object",
      properties: { arm: { enum: ["left", "right"] }, axis: { enum: ["x", "y", "z"] }, millimeters: { type: "number", minimum: -50, maximum: 50 } },
      required: ["arm", "axis", "millimeters"], additionalProperties: false,
    },
    annotations: { readOnlyHint: false, untrustedContentHint: false },
    async execute(input) {
      if (!Number.isFinite(input.millimeters) || input.millimeters === 0) throw new Error("millimeters must be non-zero");
      const translation = { x: 0, y: 0, z: 0 };
      translation[input.axis] = input.millimeters / 1000;
      state.execution = await api(`/api/robot/arms/${input.arm}/nudge`, { method: "POST", body: JSON.stringify({ translation }) });
      renderExecution();
      return { executionId: state.execution.id, status: state.execution.status };
    },
  });
}

dom.search.addEventListener("input", (event) => { state.query = event.target.value; renderSkills(); });
dom.name.addEventListener("input", () => { state.dirty = true; renderActions(); });
dom.description.addEventListener("input", () => { state.dirty = true; renderActions(); });
document.querySelector("#refresh-button").addEventListener("click", () => refresh());
document.querySelector("#new-skill-button").addEventListener("click", newSkill);
document.querySelectorAll("[data-add-action]").forEach((button) => button.addEventListener("click", () => openActionDialog(button.dataset.addAction)));
document.querySelectorAll("[data-nudge-axis]").forEach((button) => button.addEventListener("click", () => nudgeArm("translation", button.dataset.nudgeAxis, Number(button.dataset.nudgeSign))));
document.querySelectorAll("[data-nudge-rotation-axis]").forEach((button) => button.addEventListener("click", () => nudgeArm("rotation", button.dataset.nudgeRotationAxis, Number(button.dataset.nudgeSign))));
document.querySelector("#set-marker-button").addEventListener("click", setMarker);
document.querySelector("#delete-marker-button").addEventListener("click", deleteMarker);
dom.captureReference.addEventListener("change", () => { dom.captureFrame.disabled = dom.captureReference.value === "world"; });
dom.capturePose.addEventListener("click", capturePose);
dom.save.addEventListener("click", saveSkill);
dom.run.addEventListener("click", runSkill);
dom.remove.addEventListener("click", deleteSkill);
dom.cancel.addEventListener("click", cancelExecution);
dom.actionForm.addEventListener("submit", (event) => {
  event.preventDefault();
  if (event.submitter?.value === "default") commitAction();
  dom.dialog.close();
});
document.querySelectorAll("[data-dialog-close]").forEach((button) =>
  button.addEventListener("click", () => dom.dialog.close()),
);

refresh().then(registerWebMCPTools);
setInterval(() => refresh({ quiet: true }), 1500);
