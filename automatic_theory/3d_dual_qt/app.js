const THREE = window.THREE;
const showRuntimeError = (message) => {
  const box = document.createElement('div');
  box.style.position = 'fixed';
  box.style.left = '12px';
  box.style.right = '12px';
  box.style.bottom = '12px';
  box.style.zIndex = '99999';
  box.style.padding = '10px 12px';
  box.style.borderRadius = '12px';
  box.style.background = 'rgba(26, 16, 10, 0.85)';
  box.style.border = '1px solid rgba(201, 104, 43, 0.6)';
  box.style.color = '#fbd3b0';
  box.style.fontSize = '12px';
  box.style.whiteSpace = 'pre-wrap';
  box.textContent = message;
  document.body.appendChild(box);
};

const $ = (id) => document.getElementById(id);
const EPS = 1e-8;
const MAX_HISTORY = 12;

const state = {
  links: [0.6, 0.5, 0.4],
  joints: [
    { type: 'R', angleDeg: 30, axisMode: 'z', axisCustom: [0, 0, 1] },
    { type: 'R', angleDeg: -20, axisMode: 'y', axisCustom: [0, 1, 0] },
    { type: 'R', angleDeg: 40, axisMode: 'y', axisCustom: [0, 1, 0] },
    { type: 'P', displacement: 0.2, axisMode: 'x', axisCustom: [1, 0, 0] }
  ],
  error: {
    q: [1, 0, 0, 0],
    qt: [0, 0, 0, 0]
  },
  target: {
    mode: 'pose',
    position: [0.7, 0.2, 0.3],
    axis: [0, 0, 1],
    angleDeg: 45,
    q: [1, 0, 0, 0],
    qt: [0, 0.35, 0.1, 0.15]
  },
  trajectory: {
    method: 'dqlerp',
    points: 60
  },
  gamma: {
    o1: 2,
    o2: 2,
    t1: 2,
    t2: 2
  },
  iteration: {
    dt: 0.05,
    iteration: 0,
    targetIndex: 0,
    autoAdvance: true,
    running: false,
    timer: null,
    history: [],
    basePose: null,
    runIntervalMs: 140
  },
  showAxes: true,
  showTraj: true,
  showTrajError: true
};

let renderer;
let scene;
let camera;
let controls;
let armGroup;
let linkMeshes = [];
let jointSpheres = [];
let axisHelpers = [];
let targetAxes;
let nominalLine;
let errorLine;

if (!THREE) {
  showRuntimeError('Three.js 未加载成功。请检查 `3d_dual_qt/vendor/three.min.js` 是否存在。');
} else if (!THREE.OrbitControls) {
  showRuntimeError('OrbitControls 未加载成功。请检查 `3d_dual_qt/vendor/OrbitControls.js` 是否存在。');
} else {
  initUI();
  initScene();
  updateAll();
}

function initUI() {
  bindRangeNumber('l1', 'l1-num', (value) => { state.links[0] = value; updateAll(); });
  bindRangeNumber('l2', 'l2-num', (value) => { state.links[1] = value; updateAll(); });
  bindRangeNumber('l3', 'l3-num', (value) => { state.links[2] = value; updateAll(); });

  bindRangeNumber('j1-angle', 'j1-angle-num', (value) => { state.joints[0].angleDeg = value; updateAll(); });
  bindRangeNumber('j2-angle', 'j2-angle-num', (value) => { state.joints[1].angleDeg = value; updateAll(); });
  bindRangeNumber('j3-angle', 'j3-angle-num', (value) => { state.joints[2].angleDeg = value; updateAll(); });
  bindRangeNumber('j4-d', 'j4-d-num', (value) => { state.joints[3].displacement = value; updateAll(); });

  bindAxisControls(0, 'j1');
  bindAxisControls(1, 'j2');
  bindAxisControls(2, 'j3');
  bindAxisControls(3, 'j4');

  $('show-axes').addEventListener('change', (event) => {
    state.showAxes = event.target.checked;
    updateAll();
  });
  $('show-traj').addEventListener('change', (event) => {
    state.showTraj = event.target.checked;
    updateAll();
  });
  $('show-traj-error').addEventListener('change', (event) => {
    state.showTrajError = event.target.checked;
    updateAll();
  });

  bindErrorInputs();

  $('target-mode-pose').addEventListener('change', () => {
    setTargetMode('pose');
    updateAll();
  });
  $('target-mode-dq').addEventListener('change', () => {
    setTargetMode('dq');
    updateAll();
  });

  ['target-px', 'target-py', 'target-pz'].forEach((id, idx) => {
    $(id).addEventListener('input', (event) => {
      state.target.position[idx] = parseFloat(event.target.value) || 0;
      updateAll();
    });
  });
  ['target-axis-x', 'target-axis-y', 'target-axis-z'].forEach((id, idx) => {
    $(id).addEventListener('input', (event) => {
      state.target.axis[idx] = parseFloat(event.target.value) || 0;
      updateAll();
    });
  });
  $('target-angle').addEventListener('input', (event) => {
    state.target.angleDeg = parseFloat(event.target.value) || 0;
    updateAll();
  });

  ['target-qw', 'target-qx', 'target-qy', 'target-qz'].forEach((id, idx) => {
    $(id).addEventListener('input', (event) => {
      state.target.q[idx] = parseFloat(event.target.value) || 0;
      updateAll();
    });
  });
  ['target-qtw', 'target-qtx', 'target-qty', 'target-qtz'].forEach((id, idx) => {
    $(id).addEventListener('input', (event) => {
      state.target.qt[idx] = parseFloat(event.target.value) || 0;
      updateAll();
    });
  });

  $('target-from-current').addEventListener('click', () => {
    const current = computeKinematics();
    const target = current.chain.xN;
    state.target.q = [...target.q];
    state.target.qt = [...target.qt];
    state.target.mode = 'dq';
    $('target-mode-dq').checked = true;
    setTargetMode('dq');
    updateTargetInputs();
    updateAll();
  });

  $('traj-method').addEventListener('change', (event) => {
    state.trajectory.method = event.target.value;
    updateAll();
  });
  $('traj-count').addEventListener('input', (event) => {
    state.trajectory.points = clampInt(parseInt(event.target.value, 10) || 60, 10, 200);
    updateAll();
  });

  ['gamma-o1', 'gamma-o2', 'gamma-t1', 'gamma-t2'].forEach((id) => {
    $(id).addEventListener('input', () => {
      state.gamma.o1 = parseFloat($('gamma-o1').value) || 0.1;
      state.gamma.o2 = parseFloat($('gamma-o2').value) || 0.1;
      state.gamma.t1 = parseFloat($('gamma-t1').value) || 0.1;
      state.gamma.t2 = parseFloat($('gamma-t2').value) || 0.1;
      updateAll();
    });
  });

  $('jac-verify').addEventListener('click', () => {
    const result = verifyJacobian();
    $('jac-verify-result').textContent = result;
  });

  $('dt').addEventListener('input', (event) => {
    state.iteration.dt = clampFloat(parseFloat(event.target.value) || 0.05, 0.001, 1);
    updateAll();
  });
  $('target-index').addEventListener('input', (event) => {
    state.iteration.targetIndex = Math.max(0, parseInt(event.target.value, 10) || 0);
    updateAll();
  });
  $('target-auto').addEventListener('change', (event) => {
    state.iteration.autoAdvance = event.target.checked;
    updateAll();
  });
  $('step-once').addEventListener('click', () => {
    stepIteration();
  });
  $('run-toggle').addEventListener('click', () => {
    toggleRun();
  });
  $('reset-iter').addEventListener('click', () => {
    resetIteration();
  });
}

function initScene() {
  const canvas = $('scene');
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  scene = new THREE.Scene();
  scene.background = new THREE.Color('#eef3f2');

  camera = new THREE.PerspectiveCamera(50, canvas.clientWidth / canvas.clientHeight, 0.1, 50);
  camera.position.set(2.5, 1.8, 2.5);

  controls = new THREE.OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.enableZoom = false;
  const enableZoom = () => { controls.enableZoom = true; };
  const disableZoom = () => { controls.enableZoom = false; };
  canvas.addEventListener('pointerenter', enableZoom);
  canvas.addEventListener('pointerleave', disableZoom);
  canvas.addEventListener('mouseenter', enableZoom);
  canvas.addEventListener('mouseleave', disableZoom);
  if (canvas.matches(':hover')) {
    enableZoom();
  }

  const ambient = new THREE.AmbientLight(0xffffff, 0.6);
  const dir = new THREE.DirectionalLight(0xffffff, 0.8);
  dir.position.set(3, 4, 2);
  scene.add(ambient, dir);

  const grid = new THREE.GridHelper(8, 20, '#b8c3c2', '#d3dbda');
  grid.position.y = -0.001;
  scene.add(grid);

  const axes = new THREE.AxesHelper(0.4);
  scene.add(axes);

  armGroup = new THREE.Group();
  scene.add(armGroup);

  const linkMaterial = new THREE.MeshStandardMaterial({ color: '#455a64', roughness: 0.4 });
  const linkGeometry = new THREE.BoxGeometry(1, 0.06, 0.06);
  for (let i = 0; i < 4; i += 1) {
    const mesh = new THREE.Mesh(linkGeometry, linkMaterial);
    armGroup.add(mesh);
    linkMeshes.push(mesh);
  }

  const jointMaterial = new THREE.MeshStandardMaterial({ color: '#c9682b', roughness: 0.2 });
  const jointGeometry = new THREE.SphereGeometry(0.045, 18, 18);
  for (let i = 0; i < 5; i += 1) {
    const sphere = new THREE.Mesh(jointGeometry, jointMaterial);
    armGroup.add(sphere);
    jointSpheres.push(sphere);
  }

  const axisColors = ['#ff6b4a', '#31a354', '#4a7bff', '#f2a154'];
  for (let i = 0; i < 4; i += 1) {
    const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.4, axisColors[i]);
    armGroup.add(arrow);
    axisHelpers.push(arrow);
  }

  targetAxes = new THREE.AxesHelper(0.25);
  scene.add(targetAxes);

  nominalLine = buildLine('#1b6b57');
  errorLine = buildLine('#c9682b');
  scene.add(nominalLine);
  scene.add(errorLine);

  window.addEventListener('resize', onResize);
  onResize();
  animate();
}

function animate() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

function onResize() {
  const canvas = $('scene');
  const width = Math.max(2, canvas.clientWidth);
  const height = Math.max(2, canvas.clientHeight);
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function updateAll() {
  renderAll(computeKinematics());
}

function renderAll(computed) {
  updateUI(computed);
  updateScene(computed);
}

function computeKinematics() {
  const axesLocal = state.joints.map((joint) => getAxisVector(joint));
  const axesWorld = [];
  const positions = [];
  const rotations = [];

  let p = [0, 0, 0];
  let q = [1, 0, 0, 0];
  positions.push(p);
  rotations.push(q);

  const jointTransforms = [];

  for (let i = 0; i < state.joints.length; i += 1) {
    const joint = state.joints[i];
    const axisLocal = axesLocal[i];
    const axisWorld = quatRotateVec(q, axisLocal);
    axesWorld.push(axisWorld);

    let qJoint = [1, 0, 0, 0];
    let tJoint = [0, 0, 0];

    if (joint.type === 'R') {
      const theta = degToRad(joint.angleDeg);
      qJoint = quatFromAxisAngle(axisLocal, theta);
      tJoint = quatRotateVec(qJoint, [state.links[i], 0, 0]);
    } else {
      tJoint = scaleVec(axisLocal, joint.displacement);
    }

    const worldT = quatRotateVec(q, tJoint);
    p = addVec(p, worldT);
    q = quatMul(q, qJoint);

    positions.push(p);
    rotations.push(q);

    const dq = dqFromRotationTranslation(qJoint, tJoint);
    jointTransforms.push({ q: qJoint, qt: dq.qt, t: tJoint });
  }

  const x1 = jointTransforms[0];
  const x2 = jointTransforms[1];
  const x3 = jointTransforms[2];
  const x4 = { q: [1, 0, 0, 0], qt: jointTransforms[3].qt, t: jointTransforms[3].t };

  const x12 = dqMul(x1, x2);
  const x123 = dqMul(x12, x3);
  const xN = dqMul(x123, x4);

  const error = { ...state.error };
  const xReal = dqMul(xN, error);
  const xDelta = dqMul(dqInverse(xN), xReal);

  const targetGoal = getTargetDQ();
  const basePose = ensureIterationBase(xN);
  const trajectory = buildTrajectory(basePose, targetGoal, error, state.trajectory);
  const targetIndex = clampInt(state.iteration.targetIndex, 0, Math.max(0, trajectory.total - 1));
  state.iteration.targetIndex = targetIndex;
  trajectory.currentIndex = targetIndex;
  const targetSample = trajectory.samples[targetIndex] || targetGoal;
  const jacobian = computeJacobian(positions, axesWorld, positions[positions.length - 1]);
  const control = computeControl(jacobian.J, xReal, targetSample, trajectory, state.iteration.dt);

  return {
    axesLocal,
    axesWorld,
    positions,
    rotations,
    jointTransforms: [x1, x2, x3, x4],
    chain: { x1, x2, x3, x4, x12, x123, xN },
    xReal,
    xDelta,
    targetGoal,
    targetSample,
    basePose,
    trajectory,
    jacobian,
    control
  };
}

function updateUI(data) {
  updateAxisDisplay(0, data.axesLocal[0]);
  updateAxisDisplay(1, data.axesLocal[1]);
  updateAxisDisplay(2, data.axesLocal[2]);
  updateAxisDisplay(3, data.axesLocal[3]);

  updateJointDisplay(0, data.jointTransforms[0]);
  updateJointDisplay(1, data.jointTransforms[1]);
  updateJointDisplay(2, data.jointTransforms[2]);
  updateJointDisplay(3, data.jointTransforms[3]);

  updateDQ('x1', data.chain.x1);
  updateDQ('x2', data.chain.x2);
  updateDQ('x3', data.chain.x3);
  updateDQ('x4', data.chain.x4);
  updateDQ('x12', data.chain.x12);
  updateDQ('x123', data.chain.x123);
  updateDQ('xN', data.chain.xN);

  $('xN-t').textContent = fmtVec(dqToTranslation(data.chain.xN));

  $('x-real').textContent = dqToString(data.xReal);
  $('x-delta').textContent = dqToString(data.xDelta);

  $('x-d').textContent = dqToString(data.targetSample);
  $('x-d-t').textContent = fmtVec(dqToTranslation(data.targetSample));
  $('x-goal').textContent = dqToString(data.targetGoal);
  $('x-goal-t').textContent = fmtVec(dqToTranslation(data.targetGoal));

  $('jac-1').textContent = fmtVec6(data.jacobian.columns[0]);
  $('jac-2').textContent = fmtVec6(data.jacobian.columns[1]);
  $('jac-3').textContent = fmtVec6(data.jacobian.columns[2]);
  $('jac-4').textContent = fmtVec6(data.jacobian.columns[3]);

  $('kappa-o').textContent = data.control.kappaO.toFixed(3);
  $('kappa-t').textContent = data.control.kappaT.toFixed(3);
  $('qdot').textContent = fmtVec(data.control.qdot);
  $('qprime').textContent = fmtVec(data.control.qprime);

  $('overlay-xn').textContent = dqToString(data.chain.xN);
  $('overlay-xreal').textContent = dqToString(data.xReal);

  $('xn-state').textContent = dqToString(data.chain.xN);
  $('xreal-state').textContent = dqToString(data.xReal);
  $('xd-state').textContent = dqToString(data.targetSample);
  $('xtilde-state').textContent = dqToString(data.control.xTilde);
  $('ztilde-state').textContent = dqToString(data.control.zTilde);
  $('oz-vec').textContent = fmtVec(data.control.oTilde);
  $('tz-vec').textContent = fmtVec(data.control.tTilde);
  $('oz-norm').textContent = data.control.norms.o.toFixed(4);
  $('tz-norm').textContent = data.control.norms.t.toFixed(4);
  $('gamma-value').textContent = data.control.gamma.toFixed(3);
  $('term-o').textContent = fmtVec(data.control.termO);
  $('term-xi').textContent = fmtVec6(data.control.xiTerm);
  $('term-t').textContent = fmtVec(data.control.termT);
  $('control-u').textContent = fmtVec6(data.control.u);

  updateIterationUI(data);
  updateJointInputs();
  renderErrorHistory();
  updateTargetInputs();
}

function updateScene(data) {
  const positions = data.positions;
  for (let i = 0; i < linkMeshes.length; i += 1) {
    updateLinkMesh(linkMeshes[i], positions[i], positions[i + 1]);
  }
  for (let i = 0; i < jointSpheres.length; i += 1) {
    const pos = positions[i];
    jointSpheres[i].position.set(pos[0], pos[1], pos[2]);
  }

  for (let i = 0; i < axisHelpers.length; i += 1) {
    axisHelpers[i].visible = state.showAxes;
    const pos = positions[i];
    const dir = data.axesWorld[i];
    axisHelpers[i].position.set(pos[0], pos[1], pos[2]);
    axisHelpers[i].setDirection(new THREE.Vector3(dir[0], dir[1], dir[2]));
    axisHelpers[i].setLength(0.4);
  }

  const targetTranslation = dqToTranslation(data.targetSample);
  targetAxes.position.set(targetTranslation[0], targetTranslation[1], targetTranslation[2]);
  const targetQuat = data.targetSample.q;
  targetAxes.quaternion.set(targetQuat[1], targetQuat[2], targetQuat[3], targetQuat[0]);

  nominalLine.visible = state.showTraj;
  errorLine.visible = state.showTraj && state.showTrajError;
  updateLine(nominalLine, data.trajectory.points);
  updateLine(errorLine, data.trajectory.errorPoints);
}

function updateLinkMesh(mesh, start, end) {
  const startVec = new THREE.Vector3(start[0], start[1], start[2]);
  const endVec = new THREE.Vector3(end[0], end[1], end[2]);
  const dir = new THREE.Vector3().subVectors(endVec, startVec);
  const length = dir.length();
  if (length < EPS) {
    mesh.visible = false;
    return;
  }
  mesh.visible = true;
  mesh.scale.set(length, 1, 1);
  mesh.position.copy(startVec.clone().add(endVec).multiplyScalar(0.5));
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), dir.normalize());
}

function updateLine(line, points) {
  const vectors = points.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
  line.geometry.setFromPoints(vectors);
}

function buildLine(color) {
  const geometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  const material = new THREE.LineBasicMaterial({ color, linewidth: 2 });
  return new THREE.Line(geometry, material);
}

function stepIteration() {
  const before = computeKinematics();
  applyJointIncrement(before.control.qprime);
  state.iteration.iteration += 1;
  if (state.iteration.autoAdvance) {
    advanceTargetIndex(before.trajectory.total);
  }
  const after = computeKinematics();
  recordError(after.control);
  renderAll(after);
  pulseViewer();
  if (state.iteration.running && shouldStopIteration(after.control)) {
    stopRunLoop();
    updateAll();
  }
}

function toggleRun() {
  if (state.iteration.running) {
    stopRunLoop();
  } else {
    startRunLoop();
  }
  updateAll();
}

function startRunLoop() {
  if (state.iteration.running) return;
  state.iteration.running = true;
  state.iteration.timer = setInterval(() => {
    stepIteration();
  }, state.iteration.runIntervalMs);
}

function stopRunLoop() {
  state.iteration.running = false;
  if (state.iteration.timer) {
    clearInterval(state.iteration.timer);
    state.iteration.timer = null;
  }
}

function resetIteration() {
  stopRunLoop();
  const current = computeKinematics();
  state.iteration.basePose = current.chain.xN;
  state.iteration.iteration = 0;
  state.iteration.targetIndex = 0;
  state.iteration.history = [];
  renderAll(computeKinematics());
}

function ensureIterationBase(xN) {
  if (!state.iteration.basePose) {
    state.iteration.basePose = xN;
  }
  return state.iteration.basePose;
}

function advanceTargetIndex(total) {
  const maxIndex = Math.max(0, total - 1);
  state.iteration.targetIndex = Math.min(state.iteration.targetIndex + 1, maxIndex);
}

function shouldStopIteration(control) {
  return control.norms.o < 0.01 && control.norms.t < 0.01;
}

function recordError(control) {
  if (!control || !control.norms) return;
  state.iteration.history.unshift({
    iter: state.iteration.iteration,
    oNorm: control.norms.o,
    tNorm: control.norms.t
  });
  if (state.iteration.history.length > MAX_HISTORY) {
    state.iteration.history.pop();
  }
}

function renderErrorHistory() {
  const container = $('error-history');
  if (!container) return;
  container.innerHTML = '';
  if (!state.iteration.history.length) {
    const empty = document.createElement('div');
    empty.className = 'value';
    empty.textContent = '等待迭代数据…';
    container.appendChild(empty);
    return;
  }
  state.iteration.history.forEach((item) => {
    const row = document.createElement('div');
    row.className = 'error-row';
    const iter = document.createElement('div');
    iter.textContent = item.iter;
    const o = document.createElement('div');
    o.textContent = item.oNorm.toFixed(4);
    const t = document.createElement('div');
    t.textContent = item.tNorm.toFixed(4);
    row.append(iter, o, t);
    container.appendChild(row);
  });
}

function updateIterationUI(data) {
  $('dt').value = state.iteration.dt.toFixed(3);
  $('iter-count').textContent = state.iteration.iteration;
  const targetIndexInput = $('target-index');
  targetIndexInput.value = state.iteration.targetIndex;
  targetIndexInput.max = Math.max(0, data.trajectory.total - 1);
  $('target-auto').checked = state.iteration.autoAdvance;
  $('run-state').textContent = state.iteration.running ? '运行中' : '暂停';
  const runButton = $('run-toggle');
  runButton.textContent = state.iteration.running ? 'Pause' : 'Run';
  runButton.classList.toggle('run-active', state.iteration.running);
}

function updateJointInputs() {
  setInputPair('j1-angle', 'j1-angle-num', state.joints[0].angleDeg);
  setInputPair('j2-angle', 'j2-angle-num', state.joints[1].angleDeg);
  setInputPair('j3-angle', 'j3-angle-num', state.joints[2].angleDeg);
  setInputPair('j4-d', 'j4-d-num', state.joints[3].displacement);
}

function setInputPair(rangeId, numberId, value) {
  const range = $(rangeId);
  const number = $(numberId);
  if (range) range.value = value;
  if (number) number.value = value;
}

function applyJointIncrement(qprime) {
  state.joints.forEach((joint, idx) => {
    const delta = qprime[idx] || 0;
    if (joint.type === 'R') {
      const range = $(`j${idx + 1}-angle`);
      joint.angleDeg = clampToInput(joint.angleDeg + radToDeg(delta), range);
    } else {
      const range = $(`j${idx + 1}-d`);
      joint.displacement = clampToInput(joint.displacement + delta, range);
    }
  });
}

function clampToInput(value, input) {
  if (!input) return value;
  const min = parseFloat(input.min);
  const max = parseFloat(input.max);
  let next = value;
  if (!Number.isNaN(min)) next = Math.max(min, next);
  if (!Number.isNaN(max)) next = Math.min(max, next);
  return next;
}

function pulseViewer() {
  const viewer = $('viewer');
  if (!viewer) return;
  viewer.classList.remove('pulse');
  void viewer.offsetWidth;
  viewer.classList.add('pulse');
}

function bindRangeNumber(rangeId, numberId, onChange) {
  const range = $(rangeId);
  const number = $(numberId);
  range.addEventListener('input', (event) => {
    number.value = event.target.value;
    onChange(parseFloat(event.target.value));
  });
  number.addEventListener('input', (event) => {
    range.value = event.target.value;
    onChange(parseFloat(event.target.value));
  });
}

function bindAxisControls(index, prefix) {
  const mode = $(`${prefix}-axis-mode`);
  const inputs = ['x', 'y', 'z'].map((axis) => $(`${prefix}-axis-${axis}`));
  const updateAxis = () => {
    state.joints[index].axisMode = mode.value;
    inputs.forEach((input, i) => {
      state.joints[index].axisCustom[i] = parseFloat(input.value) || 0;
    });
    updateAll();
  };
  mode.addEventListener('change', updateAxis);
  inputs.forEach((input) => input.addEventListener('input', updateAxis));
}

function bindErrorInputs() {
  ['c-qw', 'c-qx', 'c-qy', 'c-qz'].forEach((id, idx) => {
    $(id).addEventListener('input', (event) => {
      state.error.q[idx] = parseFloat(event.target.value) || 0;
      updateAll();
    });
  });
  ['c-qtw', 'c-qtx', 'c-qty', 'c-qtz'].forEach((id, idx) => {
    $(id).addEventListener('input', (event) => {
      state.error.qt[idx] = parseFloat(event.target.value) || 0;
      updateAll();
    });
  });
  $('normalize-error').addEventListener('click', () => {
    state.error.q = quatNormalize(state.error.q);
    updateErrorInputs();
    updateAll();
  });
}

function setTargetMode(mode) {
  state.target.mode = mode;
  $('target-pose').classList.toggle('hidden', mode !== 'pose');
  $('target-dq').classList.toggle('hidden', mode !== 'dq');
}

function updateTargetInputs() {
  if (state.target.mode === 'pose') {
    ['target-px', 'target-py', 'target-pz'].forEach((id, idx) => {
      $(id).value = state.target.position[idx];
    });
    ['target-axis-x', 'target-axis-y', 'target-axis-z'].forEach((id, idx) => {
      $(id).value = state.target.axis[idx];
    });
    $('target-angle').value = state.target.angleDeg;
    return;
  }
  ['target-qw', 'target-qx', 'target-qy', 'target-qz'].forEach((id, idx) => {
    $(id).value = state.target.q[idx].toFixed(3);
  });
  ['target-qtw', 'target-qtx', 'target-qty', 'target-qtz'].forEach((id, idx) => {
    $(id).value = state.target.qt[idx].toFixed(3);
  });
}

function updateErrorInputs() {
  ['c-qw', 'c-qx', 'c-qy', 'c-qz'].forEach((id, idx) => {
    $(id).value = state.error.q[idx].toFixed(3);
  });
}

function updateAxisDisplay(index, axis) {
  const id = `j${index + 1}`;
  $(`${id}-axis-unit`).textContent = fmtVec(axis);
  $(`${id}-axis-pure`).textContent = `0 + 1*(${fmtVec(axis)})`;
}

function updateJointDisplay(index, dq) {
  const id = `j${index + 1}`;
  $(`${id}-q`).textContent = fmtQuat(dq.q || dq);
  $(`${id}-qt`).textContent = fmtQuat(dq.qt || dq);
}

function updateDQ(prefix, dq) {
  $(`${prefix}-q`).textContent = fmtQuat(dq.q);
  $(`${prefix}-qt`).textContent = fmtQuat(dq.qt);
}

function getAxisVector(joint) {
  if (joint.axisMode === 'x') return [1, 0, 0];
  if (joint.axisMode === 'y') return [0, 1, 0];
  if (joint.axisMode === 'z') return [0, 0, 1];
  return normalizeVec(joint.axisCustom);
}

function getTargetDQ() {
  if (state.target.mode === 'pose') {
    const axis = normalizeVec(state.target.axis);
    const angle = degToRad(state.target.angleDeg);
    const q = quatFromAxisAngle(axis, angle);
    const qt = quatScale(quatMul([0, ...state.target.position], q), 0.5);
    return { q, qt };
  }
  return {
    q: quatNormalize(state.target.q),
    qt: state.target.qt
  };
}

function buildTrajectory(xStart, xEnd, error, options) {
  const samples = [];
  const points = [];
  const errorPoints = [];
  const total = Math.max(10, options.points);

  for (let i = 0; i < total; i += 1) {
    const t = i / (total - 1);
    let x;
    if (options.method === 'slerp') {
      x = slerpPose(xStart, xEnd, t);
    } else {
      x = dqLerp(xStart, xEnd, t);
    }
    samples.push(x);
    const pos = dqToTranslation(x);
    points.push(pos);
    const xErr = dqMul(x, error);
    const posErr = dqToTranslation(xErr);
    errorPoints.push(posErr);
  }

  return { samples, points, errorPoints, total };
}

function slerpPose(a, b, t) {
  const qa = a.q;
  let qb = b.q;
  if (quatDot(qa, qb) < 0) {
    qb = qb.map((v) => -v);
  }
  const q = quatSlerp(qa, qb, t);
  const ta = dqToTranslation(a);
  const tb = dqToTranslation(b);
  const tVec = lerpVec(ta, tb, t);
  const qt = quatScale(quatMul([0, ...tVec], q), 0.5);
  return { q, qt };
}

function dqLerp(a, b, t) {
  const q = lerpQuat(a.q, b.q, t);
  const qt = lerpQuat(a.qt, b.qt, t);
  const x = { q, qt };
  return dqNormalize(x);
}

function computeJacobian(positions, axesWorld, endPos) {
  const columns = [];
  const J = Array.from({ length: 6 }, () => Array(4).fill(0));

  for (let i = 0; i < 4; i += 1) {
    const axis = axesWorld[i];
    const jointPos = positions[i];
    let omega = [0, 0, 0];
    let v = [0, 0, 0];

    if (state.joints[i].type === 'R') {
      omega = axis;
      const r = subVec(endPos, jointPos);
      v = crossVec(omega, r);
    } else {
      omega = [0, 0, 0];
      v = axis;
    }

    const col = [...omega, ...v];
    columns.push(col);
    for (let row = 0; row < 6; row += 1) {
      J[row][i] = col[row];
    }
  }

  return { J, columns };
}

function computeControl(J, xReal, xDesired, trajectory, dt) {
  const xTilde = dqMul(dqInverse(xDesired), xReal);
  const one = { q: [1, 0, 0, 0], qt: [0, 0, 0, 0] };
  const zTilde = dqSub(one, xTilde);
  const oTilde = zTilde.q.slice(1, 4);
  const tz = dqMul(zTilde, dqSub(one, zTilde));
  const tTilde = dqToTranslation(dqScale(tz, -2));

  const xiD = estimateDesiredTwist(trajectory);
  const xiTerm = transformTwist(xTilde, xiD);
  const kappaO = powNeg2(state.gamma.o1) + powNeg2(state.gamma.o2);
  const kappaT = powNeg2(state.gamma.t1) + powNeg2(state.gamma.t2);
  const gamma = Math.max(state.gamma.o1, state.gamma.o2, state.gamma.t1, state.gamma.t2);

  const termO = scaleVec(oTilde, kappaO);
  const termT = scaleVec(tTilde, -kappaT);
  const omega = addVec(termO, xiTerm.slice(0, 3));
  const v = addVec(termT, xiTerm.slice(3, 6));
  const u = [...omega, ...v];

  const Jplus = pseudoInverse(J);
  const qdot = matVecMul(Jplus, u);
  const qprime = qdot.map((value) => value * dt);

  return {
    kappaO,
    kappaT,
    gamma,
    qdot,
    qprime,
    xTilde,
    zTilde,
    oTilde,
    tTilde,
    xiD,
    xiTerm,
    termO,
    termT,
    u,
    norms: {
      o: vecNorm(oTilde),
      t: vecNorm(tTilde)
    }
  };
}

function estimateDesiredTwist(trajectory) {
  if (!trajectory.samples || trajectory.samples.length < 2) {
    return [0, 0, 0, 0, 0, 0];
  }
  const idx = clampInt(trajectory.currentIndex || 0, 0, trajectory.samples.length - 2);
  const x0 = trajectory.samples[idx];
  const x1 = trajectory.samples[idx + 1];
  const p0 = dqToTranslation(x0);
  const p1 = dqToTranslation(x1);
  const scale = Math.max(1, trajectory.samples.length - 1);
  const v = scaleVec(subVec(p1, p0), scale);
  const delta = dqMul(x1, dqInverse(x0));
  const axisAngle = quatToAxisAngle(delta.q);
  const omega = scaleVec(axisAngle.axis, axisAngle.angle * scale);
  return [...omega, ...v];
}

function transformTwist(x, xi) {
  const omega = xi.slice(0, 3);
  const v = xi.slice(3, 6);
  const omegaR = quatRotateVec(x.q, omega);
  const vR = addVec(quatRotateVec(x.q, v), crossVec(dqToTranslation(x), omegaR));
  return [...omegaR, ...vR];
}

function verifyJacobian() {
  const baseline = computeKinematics();
  const J = baseline.jacobian.columns;
  const basePose = baseline.chain.xN;
  const step = 1e-3;
  const errors = [];

  for (let i = 0; i < 4; i += 1) {
    const joint = state.joints[i];
    const original = joint.type === 'R' ? joint.angleDeg : joint.displacement;
    if (joint.type === 'R') {
      joint.angleDeg += radToDeg(step);
    } else {
      joint.displacement += step;
    }
    const updated = computeKinematics();
    const newPose = updated.chain.xN;
    const delta = dqMul(newPose, dqInverse(basePose));
    const axisAngle = quatToAxisAngle(delta.q);
    const omega = scaleVec(axisAngle.axis, axisAngle.angle / step);
    const dp = scaleVec(subVec(dqToTranslation(newPose), dqToTranslation(basePose)), 1 / step);
    const colNumeric = [...omega, ...dp];
    const colAnalytic = J[i];
    const diff = colNumeric.map((value, idx) => value - colAnalytic[idx]);
    const error = vecNormN(diff);
    errors.push(error);

    if (joint.type === 'R') {
      joint.angleDeg = original;
    } else {
      joint.displacement = original;
    }
  }

  updateAll();
  return `avg error: ${avg(errors).toFixed(4)}`;
}

function clampInt(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function clampFloat(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function degToRad(deg) {
  return (deg * Math.PI) / 180;
}

function radToDeg(rad) {
  return (rad * 180) / Math.PI;
}

function addVec(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function subVec(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function scaleVec(v, s) {
  return [v[0] * s, v[1] * s, v[2] * s];
}

function vecNorm(v) {
  return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

function vecNormN(v) {
  return Math.sqrt(v.reduce((acc, value) => acc + value * value, 0));
}

function normalizeVec(v) {
  const n = vecNorm(v);
  if (n < EPS) return [1, 0, 0];
  return scaleVec(v, 1 / n);
}

function crossVec(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  ];
}

function quatMul(a, b) {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw
  ];
}

function quatConj(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

function quatDot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

function quatNorm(q) {
  return Math.sqrt(quatDot(q, q));
}

function quatNormalize(q) {
  const n = quatNorm(q);
  if (n < EPS) return [1, 0, 0, 0];
  return q.map((v) => v / n);
}

function quatFromAxisAngle(axis, angle) {
  const nAxis = normalizeVec(axis);
  const half = angle * 0.5;
  const s = Math.sin(half);
  return [Math.cos(half), nAxis[0] * s, nAxis[1] * s, nAxis[2] * s];
}

function quatRotateVec(q, v) {
  const vq = [0, v[0], v[1], v[2]];
  const rotated = quatMul(quatMul(q, vq), quatConj(q));
  return rotated.slice(1, 4);
}

function quatToAxisAngle(q) {
  const nq = quatNormalize(q);
  const angle = 2 * Math.acos(Math.min(1, Math.max(-1, nq[0])));
  const s = Math.sqrt(1 - nq[0] * nq[0]);
  if (s < EPS) {
    return { axis: [1, 0, 0], angle: 0 };
  }
  return { axis: [nq[1] / s, nq[2] / s, nq[3] / s], angle };
}

function quatSlerp(a, b, t) {
  let cosHalf = quatDot(a, b);
  let qb = b;
  if (cosHalf < 0) {
    qb = b.map((v) => -v);
    cosHalf = -cosHalf;
  }
  if (cosHalf > 0.9995) {
    return quatNormalize(lerpQuat(a, qb, t));
  }
  const halfTheta = Math.acos(cosHalf);
  const sinHalf = Math.sqrt(1 - cosHalf * cosHalf);
  const ratioA = Math.sin((1 - t) * halfTheta) / sinHalf;
  const ratioB = Math.sin(t * halfTheta) / sinHalf;
  return [
    a[0] * ratioA + qb[0] * ratioB,
    a[1] * ratioA + qb[1] * ratioB,
    a[2] * ratioA + qb[2] * ratioB,
    a[3] * ratioA + qb[3] * ratioB
  ];
}

function lerpQuat(a, b, t) {
  return [
    a[0] + (b[0] - a[0]) * t,
    a[1] + (b[1] - a[1]) * t,
    a[2] + (b[2] - a[2]) * t,
    a[3] + (b[3] - a[3]) * t
  ];
}

function dqFromRotationTranslation(q, t) {
  const qt = quatScale(quatMul([0, ...t], q), 0.5);
  return { q, qt };
}

function dqToTranslation(x) {
  const tQuat = quatMul(x.qt, quatConj(x.q));
  return scaleVec(tQuat.slice(1, 4), 2);
}

function dqMul(a, b) {
  return {
    q: quatMul(a.q, b.q),
    qt: quatAdd(quatMul(a.q, b.qt), quatMul(a.qt, b.q))
  };
}

function dqScale(x, s) {
  return { q: quatScale(x.q, s), qt: quatScale(x.qt, s) };
}

function dqAdd(a, b) {
  return { q: quatAdd(a.q, b.q), qt: quatAdd(a.qt, b.qt) };
}

function dqSub(a, b) {
  return { q: quatSub(a.q, b.q), qt: quatSub(a.qt, b.qt) };
}

function dqNormalize(x) {
  const n = quatNorm(x.q);
  if (n < EPS) {
    return { q: [1, 0, 0, 0], qt: [0, 0, 0, 0] };
  }
  return { q: quatScale(x.q, 1 / n), qt: quatScale(x.qt, 1 / n) };
}

function dqInverse(x) {
  const qConj = quatConj(x.q);
  const qtInv = quatScale(quatMul(quatMul(qConj, x.qt), qConj), -1);
  return { q: qConj, qt: qtInv };
}

function quatAdd(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]];
}

function quatSub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]];
}

function quatScale(q, s) {
  return [q[0] * s, q[1] * s, q[2] * s, q[3] * s];
}

function lerpVec(a, b, t) {
  return [
    a[0] + (b[0] - a[0]) * t,
    a[1] + (b[1] - a[1]) * t,
    a[2] + (b[2] - a[2]) * t
  ];
}

function fmtVec(v) {
  return `[${v.map((n) => n.toFixed(3)).join(', ')}]`;
}

function fmtQuat(q) {
  return `[${q.map((n) => n.toFixed(3)).join(', ')}]`;
}

function fmtVec6(v) {
  return `[${v.map((n) => n.toFixed(3)).join(', ')}]`;
}

function dqToString(x) {
  return `{q:${fmtQuat(x.q)}, qt:${fmtQuat(x.qt)}}`;
}

function powNeg2(value) {
  if (value <= 0) return 0;
  return 1 / (value * value);
}

function pseudoInverse(J) {
  const JT = matTranspose(J);
  let JJt = matMul(J, JT);
  JJt = matAdd(JJt, matScale(identityMatrix(JJt.length), 1e-6));
  const JJtInv = matInverse(JJt);
  return matMul(JT, JJtInv);
}

function matMul(A, B) {
  const rows = A.length;
  const cols = B[0].length;
  const shared = B.length;
  const result = Array.from({ length: rows }, () => Array(cols).fill(0));
  for (let i = 0; i < rows; i += 1) {
    for (let k = 0; k < shared; k += 1) {
      for (let j = 0; j < cols; j += 1) {
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return result;
}

function matVecMul(A, v) {
  const rows = A.length;
  const result = Array(rows).fill(0);
  for (let i = 0; i < rows; i += 1) {
    let sum = 0;
    for (let j = 0; j < v.length; j += 1) {
      sum += A[i][j] * v[j];
    }
    result[i] = sum;
  }
  return result;
}

function matTranspose(A) {
  const rows = A.length;
  const cols = A[0].length;
  const result = Array.from({ length: cols }, () => Array(rows).fill(0));
  for (let i = 0; i < rows; i += 1) {
    for (let j = 0; j < cols; j += 1) {
      result[j][i] = A[i][j];
    }
  }
  return result;
}

function matAdd(A, B) {
  return A.map((row, i) => row.map((v, j) => v + B[i][j]));
}

function matScale(A, s) {
  return A.map((row) => row.map((v) => v * s));
}

function identityMatrix(n) {
  const I = Array.from({ length: n }, () => Array(n).fill(0));
  for (let i = 0; i < n; i += 1) {
    I[i][i] = 1;
  }
  return I;
}

function matInverse(A) {
  const n = A.length;
  const M = A.map((row, i) => [...row, ...identityMatrix(n)[i]]);

  for (let i = 0; i < n; i += 1) {
    let pivot = i;
    for (let j = i + 1; j < n; j += 1) {
      if (Math.abs(M[j][i]) > Math.abs(M[pivot][i])) {
        pivot = j;
      }
    }
    if (Math.abs(M[pivot][i]) < EPS) {
      return identityMatrix(n);
    }
    [M[i], M[pivot]] = [M[pivot], M[i]];
    const pivotVal = M[i][i];
    for (let j = 0; j < 2 * n; j += 1) {
      M[i][j] /= pivotVal;
    }
    for (let r = 0; r < n; r += 1) {
      if (r !== i) {
        const factor = M[r][i];
        for (let c = 0; c < 2 * n; c += 1) {
          M[r][c] -= factor * M[i][c];
        }
      }
    }
  }
  return M.map((row) => row.slice(n));
}

function avg(values) {
  if (!values.length) return 0;
  return values.reduce((acc, v) => acc + v, 0) / values.length;
}
