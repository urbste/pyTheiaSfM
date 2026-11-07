// pyTheia browser viewer (inline ES module, generated into viewer.html).
// three.js r160 and OrbitControls are embedded as base64 text and loaded through blob URLs,
// so the page needs no network access. Scene data is read from #pytheia-scene (JSON with
// base64 float32/uint8 buffers, see pytheia/viewer/scene.py for the layout).
// A scene with a "compare" entry carries a second reconstruction, already aligned to the
// first (Sim(3)). Each reconstruction is a "set" with its own points, cameras and objects.

const $ = (id) => document.getElementById(id);
const esc = (s) =>
  String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]);
const fmt = (x, d = 2) => (x === null || x === undefined || !Number.isFinite(x) ? '-' : Number(x).toFixed(d));
const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

function b64Bytes(b64) {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return bytes;
}

function b64Text(id) {
  return new TextDecoder('utf-8').decode(b64Bytes($(id).textContent.trim()));
}

function decodeArray(desc) {
  const bytes = b64Bytes(desc.data);
  if (desc.dtype === 'uint8') return bytes;
  return new Float32Array(bytes.buffer);
}

async function loadThree() {
  const threeURL = URL.createObjectURL(new Blob([b64Text('pytheia-three-b64')], { type: 'text/javascript' }));
  const THREE = await import(threeURL);
  const orbitSrc = b64Text('pytheia-orbit-b64').replace(/from\s+['"]three['"]/g, 'from "' + threeURL + '"');
  const orbitURL = URL.createObjectURL(new Blob([orbitSrc], { type: 'text/javascript' }));
  const orbit = await import(orbitURL);
  return { THREE, OrbitControls: orbit.OrbitControls };
}

// viridis-like ramp, t in [0, 1]
const RAMP = [
  [0.267, 0.005, 0.329],
  [0.231, 0.322, 0.545],
  [0.128, 0.567, 0.551],
  [0.369, 0.789, 0.383],
  [0.993, 0.906, 0.144],
];
function ramp(t) {
  t = Math.min(1, Math.max(0, t));
  const f = t * (RAMP.length - 1);
  const i = Math.min(RAMP.length - 2, Math.floor(f));
  const u = f - i;
  return [0, 1, 2].map((k) => RAMP[i][k] * (1 - u) + RAMP[i + 1][k] * u);
}

// per set: registered cameras, unregistered cameras, point tint (compare mode, RGB only)
const PALETTE = [
  { reg: [0.29, 0.64, 1.0], unreg: [1.0, 0.48, 0.27], tint: null, traj: 0x7fc8ff, label: 'A' },
  { reg: [1.0, 0.42, 0.84], unreg: [1.0, 0.48, 0.27], tint: [1.0, 0.42, 0.84], traj: 0xff7ad8, label: 'B' },
];

function naturalCompare(a, b) {
  return String(a).localeCompare(String(b), undefined, { numeric: true });
}

// Decode one reconstruction payload (points, observations, cameras, edges) into typed arrays.
function decodeSet(S, idx) {
  const cams = S.cameras;
  const NP = S.points.xyz.shape[0];
  const NV = cams.center.shape[0];
  const set = {
    idx,
    NP,
    NV,
    stats: S.stats,
    bounds: S.bounds,
    xyz: decodeArray(S.points.xyz),
    rgb: decodeArray(S.points.rgb),
    trackLen: decodeArray(S.points.track_length),
    perr: decodeArray(S.points.reprojection_error),
    trackIds: decodeArray(S.points.track_id),
    pOff: Uint32Array.from(decodeArray(S.points.obs_offsets)),
    oView: Uint32Array.from(decodeArray(S.observations.view)),
    oX: decodeArray(S.observations.x),
    oY: decodeArray(S.observations.y),
    oRes: decodeArray(S.observations.residual),
    camC: decodeArray(cams.center),
    camK: decodeArray(cams.intrinsics),
    camSize: decodeArray(cams.image_size),
    camFr: decodeArray(cams.frustum),
    camEst: decodeArray(cams.estimated),
    camNObs: decodeArray(cams.num_observations),
    camErr: decodeArray(cams.mean_reprojection_error),
    camVid: decodeArray(cams.view_id),
    names: cams.names,
    thumbs: cams.thumbnails,
    edges: S.edges
      ? { pairs: decodeArray(S.edges.pairs), weight: decodeArray(S.edges.weight), count: S.edges.weight.shape[0] }
      : null,
  };
  // camera -> points it observes (all points, not only the ones that pass the filters)
  set.viewPoints = Array.from({ length: NV }, () => []);
  for (let p = 0; p < NP; p++) {
    for (let k = set.pOff[p]; k < set.pOff[p + 1]; k++) set.viewPoints[set.oView[k]].push(p);
  }
  return set;
}

function main(THREE, OrbitControls) {
  THREE.ColorManagement.enabled = false; // colors are raw sRGB values

  // ---- data ---------------------------------------------------------------
  const SCENE = JSON.parse($('pytheia-scene').textContent);
  const sets = [decodeSet(SCENE, 0)];
  const CMP = SCENE.compare || null;
  if (CMP) sets.push(decodeSet(CMP.scene, 1));
  const isCompare = sets.length > 1;
  const A = sets[0];
  const cmpErr = CMP
    ? {
        matched: decodeArray(CMP.cameras.matched_b),
        cerr: decodeArray(CMP.cameras.center_error),
        rerr: decodeArray(CMP.cameras.rotation_error_deg),
      }
    : null;
  document.documentElement.dataset.compare = isCompare ? '1' : '0';

  // ---- global ranges and framing ------------------------------------------
  let trackMin = Infinity, trackMax = 1;
  const errList = [];
  const bmin = [...SCENE.bounds.min], bmax = [...SCENE.bounds.max];
  const cen = [0, 0, 0];
  let nAll = 0;
  for (const s of sets) {
    for (let p = 0; p < s.NP; p++) {
      trackMin = Math.min(trackMin, s.trackLen[p]);
      trackMax = Math.max(trackMax, s.trackLen[p]);
      if (Number.isFinite(s.perr[p])) errList.push(s.perr[p]);
      for (let j = 0; j < 3; j++) cen[j] += s.xyz[3 * p + j];
      nAll++;
    }
    for (let j = 0; j < 3; j++) {
      bmin[j] = Math.min(bmin[j], s.bounds.min[j]);
      bmax[j] = Math.max(bmax[j], s.bounds.max[j]);
    }
    for (let i = 0; i < s.NV; i++) {
      for (let j = 0; j < 3; j++) {
        bmin[j] = Math.min(bmin[j], s.camC[3 * i + j]);
        bmax[j] = Math.max(bmax[j], s.camC[3 * i + j]);
      }
    }
  }
  if (!Number.isFinite(trackMin)) trackMin = 1;
  const errSorted = Float32Array.from(errList).sort();
  const errMaxAll = errSorted.length ? errSorted[errSorted.length - 1] : 1;
  const errP99 = errSorted.length ? errSorted[Math.floor(0.99 * (errSorted.length - 1))] : 1;
  const errNorm = errP99 > 0 ? errP99 : 1;
  const centroid = nAll
    ? [cen[0] / nAll, cen[1] / nAll, cen[2] / nAll]
    : [0, 1, 2].map((j) => (bmin[j] + bmax[j]) / 2);

  // ---- renderer and scene -------------------------------------------------
  const host = $('view');
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.domElement.style.width = '100%';
  renderer.domElement.style.height = '100%';
  renderer.domElement.style.display = 'block';
  host.insertBefore(renderer.domElement, host.firstChild);
  const canvas = renderer.domElement;

  const scene3 = new THREE.Scene();
  const center = new THREE.Vector3((bmin[0] + bmax[0]) / 2, (bmin[1] + bmax[1]) / 2, (bmin[2] + bmax[2]) / 2);
  let radius = Math.hypot(bmax[0] - bmin[0], bmax[1] - bmin[1], bmax[2] - bmin[2]) / 2;
  if (!(radius > 0)) radius = 1;
  const camera = new THREE.PerspectiveCamera(50, 1, radius * 1e-3, radius * 1e3);
  const defaultPos = new THREE.Vector3(center.x, center.y + 0.3 * radius, center.z + 2.2 * radius);
  camera.position.copy(defaultPos);
  const controls = new OrbitControls(camera, canvas);
  controls.target.copy(center);
  controls.enableDamping = true;
  controls.screenSpacePanning = true;
  controls.update();

  // helper: replace a buffer attribute and let three recompute the bounding sphere
  function setAttr(geom, name, data, size) {
    geom.setAttribute(name, new THREE.BufferAttribute(data, size));
    geom.boundingSphere = null;
  }

  // highlights: points observed by the selected camera, selected point, measurement
  const obsGeom = new THREE.BufferGeometry();
  const obsObj = new THREE.Points(obsGeom, new THREE.PointsMaterial({ size: 4, sizeAttenuation: false, color: 0xffd23f }));
  scene3.add(obsObj);
  const selGeom = new THREE.BufferGeometry();
  const selObj = new THREE.Points(selGeom, new THREE.PointsMaterial({ size: 10, sizeAttenuation: false, color: 0xff3fd2 }));
  scene3.add(selObj);
  const measGeom = new THREE.BufferGeometry();
  const measPts = new THREE.Points(measGeom, new THREE.PointsMaterial({ size: 9, sizeAttenuation: false, color: 0xffffff }));
  scene3.add(measPts);
  const measLineGeom = new THREE.BufferGeometry();
  const measLine = new THREE.Line(measLineGeom, new THREE.LineBasicMaterial({ color: 0xffffff }));
  scene3.add(measLine);

  // per set objects
  for (const s of sets) {
    const pal = PALETTE[s.idx];
    s.pointGeom = new THREE.BufferGeometry();
    s.pointMat = new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, vertexColors: true });
    s.pointsObj = new THREE.Points(s.pointGeom, s.pointMat);
    scene3.add(s.pointsObj);

    // cameras: frusta (8 segments per camera, 16 vertices) and centers (points)
    s.frusGeom = new THREE.BufferGeometry();
    const frusPos = new Float32Array(s.NV * 48);
    s.frusCol = new Float32Array(s.NV * 48);
    s.centGeom = new THREE.BufferGeometry();
    const centPos = new Float32Array(s.NV * 3);
    s.centCol = new Float32Array(s.NV * 3);
    for (let i = 0; i < s.NV; i++) {
      centPos.set(s.camC.subarray(3 * i, 3 * i + 3), 3 * i);
      const c = [s.camC[3 * i], s.camC[3 * i + 1], s.camC[3 * i + 2]];
      const corner = (k) => [s.camFr[i * 12 + 3 * k], s.camFr[i * 12 + 3 * k + 1], s.camFr[i * 12 + 3 * k + 2]];
      const segs = [];
      for (let k = 0; k < 4; k++) segs.push([c, corner(k)]);
      for (let k = 0; k < 4; k++) segs.push([corner(k), corner((k + 1) % 4)]);
      segs.forEach((seg, j) => {
        frusPos.set(seg[0], i * 48 + j * 6);
        frusPos.set(seg[1], i * 48 + j * 6 + 3);
      });
    }
    setAttr(s.frusGeom, 'position', frusPos, 3);
    setAttr(s.frusGeom, 'color', s.frusCol, 3);
    setAttr(s.centGeom, 'position', centPos, 3);
    setAttr(s.centGeom, 'color', s.centCol, 3);
    s.frusObj = new THREE.LineSegments(s.frusGeom, new THREE.LineBasicMaterial({ vertexColors: true }));
    s.centObj = new THREE.Points(s.centGeom, new THREE.PointsMaterial({ size: 7, sizeAttenuation: false, vertexColors: true }));
    scene3.add(s.frusObj);
    scene3.add(s.centObj);

    // camera trajectory (filled by buildTraj)
    s.trajGeom = new THREE.BufferGeometry();
    s.trajObj = new THREE.Line(s.trajGeom, new THREE.LineBasicMaterial({ color: pal.traj }));
    s.trajCount = 0;
    scene3.add(s.trajObj);

    // view graph
    s.graphObj = null;
    if (s.edges && s.edges.count > 0) {
      const gpos = new Float32Array(s.edges.count * 6);
      const gcol = new Float32Array(s.edges.count * 6);
      let wmax = 0;
      for (let e = 0; e < s.edges.count; e++) wmax = Math.max(wmax, s.edges.weight[e]);
      for (let e = 0; e < s.edges.count; e++) {
        const a = s.edges.pairs[2 * e], b = s.edges.pairs[2 * e + 1];
        gpos.set(s.camC.subarray(3 * a, 3 * a + 3), e * 6);
        gpos.set(s.camC.subarray(3 * b, 3 * b + 3), e * 6 + 3);
        const w = 0.2 + 0.8 * (s.edges.weight[e] / (wmax || 1));
        for (let j = 0; j < 6; j++) gcol[e * 6 + j] = j % 3 === 2 ? 0.3 * w + 0.2 : w * 0.45;
      }
      const gg = new THREE.BufferGeometry();
      setAttr(gg, 'position', gpos, 3);
      setAttr(gg, 'color', gcol, 3);
      s.graphObj = new THREE.LineSegments(gg, new THREE.LineBasicMaterial({ vertexColors: true }));
      scene3.add(s.graphObj);
    }
    s.colors = new Float32Array(s.NP * 3);
    s.kept = new Uint32Array(0);
  }

  // ---- state --------------------------------------------------------------
  const state = {
    colorMode: 'rgb',
    pointSize: 2,
    minTrack: 1,
    maxErr: errMaxAll,
    showPoints: true,
    showCams: true,
    showFrusta: true,
    showUnreg: true,
    showGraph: false,
    showTraj: false,
    trajOrder: 'name',
    sets: sets.map(() => true),
    sel: null, // { type: 'cam' | 'point', set, idx }
    sortKey: 'name',
    sortDir: 1,
    fly: false,
    measure: false,
    meas: [], // [{ pos: [x, y, z], label }]
  };

  // ---- colors -------------------------------------------------------------
  let depthNorm = 1;
  let depthLabel = '';

  function depthReference() {
    const sel = state.sel;
    if (sel && sel.type === 'cam') {
      const s = sets[sel.set];
      const i = sel.idx;
      return { pos: [s.camC[3 * i], s.camC[3 * i + 1], s.camC[3 * i + 2]], label: 'camera ' + s.names[i] };
    }
    return { pos: centroid, label: 'scene centroid' };
  }

  function computeColors() {
    const lo = Math.log(Math.max(1, trackMin));
    const hi = Math.log(Math.max(1, trackMax));
    let ref = null;
    if (state.colorMode === 'depth') {
      ref = depthReference();
      depthLabel = ref.label;
      let total = 0;
      for (const s of sets) total += s.NP;
      const dist = new Float32Array(total);
      let k = 0;
      for (const s of sets) {
        for (let p = 0; p < s.NP; p++) {
          dist[k++] = Math.hypot(
            s.xyz[3 * p] - ref.pos[0],
            s.xyz[3 * p + 1] - ref.pos[1],
            s.xyz[3 * p + 2] - ref.pos[2]
          );
        }
      }
      dist.sort();
      const p99 = total ? dist[Math.floor(0.99 * (total - 1))] : 1;
      depthNorm = p99 > 0 ? p99 : 1;
    }
    for (const s of sets) {
      const col = new Float32Array(s.NP * 3);
      const tint = isCompare && s.idx > 0 ? PALETTE[s.idx].tint : null;
      for (let p = 0; p < s.NP; p++) {
        let c;
        if (state.colorMode === 'track') {
          const t = hi > lo ? (Math.log(Math.max(1, s.trackLen[p])) - lo) / (hi - lo) : 0.5;
          c = ramp(t);
        } else if (state.colorMode === 'error') {
          const e = s.perr[p];
          c = Number.isFinite(e) ? ramp(e / errNorm) : [0.5, 0.5, 0.5];
        } else if (state.colorMode === 'depth') {
          const d = Math.hypot(
            s.xyz[3 * p] - ref.pos[0],
            s.xyz[3 * p + 1] - ref.pos[1],
            s.xyz[3 * p + 2] - ref.pos[2]
          );
          c = ramp(d / depthNorm);
        } else if (tint) {
          c = tint;
        } else {
          c = [s.rgb[3 * p] / 255, s.rgb[3 * p + 1] / 255, s.rgb[3 * p + 2] / 255];
        }
        col[3 * p] = c[0];
        col[3 * p + 1] = c[1];
        col[3 * p + 2] = c[2];
      }
      s.colors = col;
    }
  }

  // ---- points and filters -------------------------------------------------
  function passes(s, p) {
    if (s.trackLen[p] < state.minTrack) return false;
    if (state.maxErr < errMaxAll) {
      const e = s.perr[p];
      if (!Number.isFinite(e) || e > state.maxErr) return false;
    }
    return true;
  }

  function rebuildPoints(s) {
    const kept = [];
    for (let p = 0; p < s.NP; p++) if (passes(s, p)) kept.push(p);
    s.kept = Uint32Array.from(kept);
    const pos = new Float32Array(kept.length * 3);
    const col = new Float32Array(kept.length * 3);
    for (let i = 0; i < kept.length; i++) {
      const p = kept[i];
      pos.set(s.xyz.subarray(3 * p, 3 * p + 3), 3 * i);
      col.set(s.colors.subarray(3 * p, 3 * p + 3), 3 * i);
    }
    setAttr(s.pointGeom, 'position', pos, 3);
    setAttr(s.pointGeom, 'color', col, 3);
  }

  function rebuildAllPoints() {
    sets.forEach(rebuildPoints);
    let shown = 0, total = 0;
    for (const s of sets) {
      total += s.NP;
      if (state.sets[s.idx]) shown += s.kept.length;
    }
    $('filter-count').textContent = shown.toLocaleString() + ' / ' + total.toLocaleString() + ' points shown';
  }

  // ---- cameras ------------------------------------------------------------
  function cameraColor(s, i) {
    const sel = state.sel;
    if (sel && sel.type === 'cam' && sel.set === s.idx && sel.idx === i) return [1.0, 1.0, 1.0];
    if (!s.camEst[i]) return state.showUnreg ? PALETTE[s.idx].unreg : [0.5, 0.5, 0.5];
    return PALETTE[s.idx].reg;
  }

  function refreshCameraColors() {
    for (const s of sets) {
      for (let i = 0; i < s.NV; i++) {
        const c = cameraColor(s, i);
        for (let j = 0; j < 16; j++) {
          s.frusCol[i * 48 + j * 3] = c[0];
          s.frusCol[i * 48 + j * 3 + 1] = c[1];
          s.frusCol[i * 48 + j * 3 + 2] = c[2];
        }
        s.centCol[3 * i] = c[0];
        s.centCol[3 * i + 1] = c[1];
        s.centCol[3 * i + 2] = c[2];
      }
      s.frusGeom.attributes.color.needsUpdate = true;
      s.centGeom.attributes.color.needsUpdate = true;
    }
  }

  function buildTraj(s) {
    const idx = [];
    for (let i = 0; i < s.NV; i++) if (s.camEst[i]) idx.push(i);
    if (state.trajOrder === 'id') idx.sort((a, b) => s.camVid[a] - s.camVid[b]);
    else idx.sort((a, b) => naturalCompare(s.names[a], s.names[b]));
    const pos = new Float32Array(idx.length * 3);
    idx.forEach((i, k) => pos.set(s.camC.subarray(3 * i, 3 * i + 3), 3 * k));
    setAttr(s.trajGeom, 'position', pos, 3);
    s.trajCount = idx.length;
  }

  function updateSelectionObjects() {
    const sel = state.sel;
    if (sel && sel.type === 'cam') {
      const s = sets[sel.set];
      const pts = s.viewPoints[sel.idx];
      const pos = new Float32Array(pts.length * 3);
      pts.forEach((p, k) => pos.set(s.xyz.subarray(3 * p, 3 * p + 3), 3 * k));
      setAttr(obsGeom, 'position', pos, 3);
    } else {
      setAttr(obsGeom, 'position', new Float32Array(0), 3);
    }
    if (sel && sel.type === 'point') {
      const s = sets[sel.set];
      setAttr(selGeom, 'position', s.xyz.slice(3 * sel.idx, 3 * sel.idx + 3), 3);
    } else {
      setAttr(selGeom, 'position', new Float32Array(0), 3);
    }
  }

  // ---- projection and picking (screen space) ------------------------------
  function screenMatrix() {
    camera.updateMatrixWorld();
    return new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse).elements;
  }

  function project(me, x, y, z, W, H) {
    const cx = me[0] * x + me[4] * y + me[8] * z + me[12];
    const cy = me[1] * x + me[5] * y + me[9] * z + me[13];
    const cw = me[3] * x + me[7] * y + me[11] * z + me[15];
    if (!(cw > 0)) return null;
    return [((cx / cw + 1) / 2) * W, ((1 - cy / cw) / 2) * H];
  }

  // returns { type, set, idx, pos } for the nearest camera (12 px) or point (6 px), or null
  function pickAt(sx, sy) {
    const W = canvas.clientWidth, H = canvas.clientHeight;
    const me = screenMatrix();
    if (state.showCams) {
      let best = null, bestD = 144;
      for (const s of sets) {
        if (!state.sets[s.idx]) continue;
        for (let i = 0; i < s.NV; i++) {
          const q = project(me, s.camC[3 * i], s.camC[3 * i + 1], s.camC[3 * i + 2], W, H);
          if (!q) continue;
          const d = (q[0] - sx) ** 2 + (q[1] - sy) ** 2;
          if (d < bestD) {
            bestD = d;
            best = { type: 'cam', set: s.idx, idx: i, pos: [s.camC[3 * i], s.camC[3 * i + 1], s.camC[3 * i + 2]] };
          }
        }
      }
      if (best) return best;
    }
    if (state.showPoints) {
      let best = null, bestD = 36;
      for (const s of sets) {
        if (!state.sets[s.idx]) continue;
        for (let k = 0; k < s.kept.length; k++) {
          const p = s.kept[k];
          const q = project(me, s.xyz[3 * p], s.xyz[3 * p + 1], s.xyz[3 * p + 2], W, H);
          if (!q) continue;
          const d = (q[0] - sx) ** 2 + (q[1] - sy) ** 2;
          if (d < bestD) {
            bestD = d;
            best = { type: 'point', set: s.idx, idx: p, pos: [s.xyz[3 * p], s.xyz[3 * p + 1], s.xyz[3 * p + 2]] };
          }
        }
      }
      if (best) return best;
    }
    return null;
  }

  // ---- info panel ---------------------------------------------------------
  function clearInfo(msg) {
    $('info-body').innerHTML = '<div class="muted">' + esc(msg || 'Click a camera or a point.') + '</div>';
  }

  // A-view index for a B camera (or B index for an A camera) through the alignment
  function matchOf(s, i) {
    if (!cmpErr) return -1;
    if (s.idx === 0) return cmpErr.matched[i];
    for (let a = 0; a < A.NV; a++) if (cmpErr.matched[a] === i) return a;
    return -1;
  }

  function setTag(s) {
    return isCompare ? '[' + PALETTE[s.idx].label + '] ' : '';
  }

  function showCamera(s, i) {
    const obs = s.viewPoints[i].length;
    const k = [s.camK[4 * i], s.camK[4 * i + 1], s.camK[4 * i + 2], s.camK[4 * i + 3]];
    const sz = [s.camSize[2 * i], s.camSize[2 * i + 1]];
    const c = [s.camC[3 * i], s.camC[3 * i + 1], s.camC[3 * i + 2]];
    const rows = [
      '<div class="title">' + setTag(s) + 'Camera ' + esc(s.names[i]) + '</div>',
      '<table class="kv">',
      '<tr><th>view id</th><td>' + esc(s.camVid[i]) + '</td></tr>',
      '<tr><th>registered</th><td>' + (s.camEst[i] ? 'yes' : '<span class="warn">no</span>') + '</td></tr>',
      '<tr><th>image size</th><td>' + fmt(sz[0], 0) + ' x ' + fmt(sz[1], 0) + '</td></tr>',
      '<tr><th>focal fx, fy</th><td>' + fmt(k[0], 1) + ', ' + fmt(k[1], 1) + '</td></tr>',
      '<tr><th>principal pt</th><td>' + fmt(k[2], 1) + ', ' + fmt(k[3], 1) + '</td></tr>',
      '<tr><th>center</th><td>' + c.map((v) => fmt(v, 3)).join(', ') + '</td></tr>',
      '<tr><th>observed points</th><td>' + obs.toLocaleString() + ' (shown in yellow)</td></tr>',
      '<tr><th>mean error</th><td>' + fmt(s.camErr[i], 3) + ' px</td></tr>',
    ];
    if (isCompare) {
      const j = matchOf(s, i);
      if (j >= 0) {
        const other = sets[1 - s.idx];
        rows.push('<tr><th>matched view</th><td>' + esc(other.names[j]) + ' in ' + PALETTE[other.idx].label + '</td></tr>');
        if (s.idx === 0) {
          rows.push('<tr><th>center error</th><td>' + fmt(cmpErr.cerr[i], 4) + ' scene units</td></tr>');
          rows.push('<tr><th>rotation error</th><td>' + fmt(cmpErr.rerr[i], 3) + ' deg</td></tr>');
        }
      } else {
        rows.push('<tr><th>matched view</th><td class="muted">none (not registered in both)</td></tr>');
      }
    }
    rows.push('</table>', '<div id="thumb"></div>');
    $('info-body').innerHTML = rows.join('');
    if (s.thumbs[i]) {
      const img = new Image();
      img.alt = s.names[i];
      img.src = s.thumbs[i];
      $('thumb').appendChild(img);
    }
  }

  function showPoint(s, p) {
    const html = [
      '<div class="title">' + setTag(s) + 'Point, track ' + esc(fmt(s.trackIds[p], 0)) + '</div>',
      '<table class="kv">',
      '<tr><th>position</th><td>' + [0, 1, 2].map((j) => fmt(s.xyz[3 * p + j], 4)).join(', ') + '</td></tr>',
      '<tr><th>color</th><td><span class="swatch" style="background:rgb(' + s.rgb[3 * p] + ',' + s.rgb[3 * p + 1] + ',' + s.rgb[3 * p + 2] + ')"></span> ' +
        s.rgb[3 * p] + ', ' + s.rgb[3 * p + 1] + ', ' + s.rgb[3 * p + 2] + '</td></tr>',
      '<tr><th>track length</th><td>' + fmt(s.trackLen[p], 0) + ' views</td></tr>',
      '<tr><th>mean error</th><td>' + fmt(s.perr[p], 3) + ' px</td></tr>',
      '</table>',
      '<table class="grid"><thead><tr><th>view</th><th>x</th><th>y</th><th>residual px</th></tr></thead><tbody>',
    ];
    for (let k = s.pOff[p]; k < s.pOff[p + 1]; k++) {
      const v = s.oView[k];
      html.push(
        '<tr data-cam="' + v + '"><td>' + esc(s.names[v]) + '</td><td>' + fmt(s.oX[k], 1) + '</td><td>' +
          fmt(s.oY[k], 1) + '</td><td>' + fmt(s.oRes[k], 3) + '</td></tr>'
      );
    }
    html.push('</tbody></table>');
    $('info-body').innerHTML = html.join('');
    $('info-body').querySelectorAll('tr[data-cam]').forEach((tr) =>
      tr.addEventListener('click', () => select('cam', Number(tr.dataset.cam), s.idx))
    );
  }

  function showMeasure() {
    const [a, b] = state.meas;
    const d = Math.hypot(a.pos[0] - b.pos[0], a.pos[1] - b.pos[1], a.pos[2] - b.pos[2]);
    const xyzText = (p) => p.map((v) => fmt(v, 3)).join(', ');
    $('info-body').innerHTML = [
      '<div class="title">Measurement</div>',
      '<table class="kv">',
      '<tr><th>distance</th><td><b>' + fmt(d, 4) + '</b> scene units</td></tr>',
      '<tr><th>from</th><td>' + esc(a.label) + ' at ' + xyzText(a.pos) + '</td></tr>',
      '<tr><th>to</th><td>' + esc(b.label) + ' at ' + xyzText(b.pos) + '</td></tr>',
      '</table>',
      '<div class="muted">Click again to start a new measurement. Clear removes it.</div>',
    ].join('');
  }

  function showDefaultInfo() {
    if (state.meas.length === 2) showMeasure();
    else clearInfo(state.measure ? 'Measure: click two points (cameras or points).' : null);
  }

  // ---- selection ----------------------------------------------------------
  function select(type, idx, setIdx = 0) {
    state.sel = type ? { type, set: setIdx, idx } : null;
    refreshCameraColors();
    updateSelectionObjects();
    if (state.colorMode === 'depth') {
      computeColors();
      rebuildAllPoints();
      updateLegend();
    }
    renderTable();
    if (type === 'cam') showCamera(sets[setIdx], idx);
    else if (type === 'point') showPoint(sets[setIdx], idx);
    else showDefaultInfo();
  }

  // ---- views table (reconstruction A) -------------------------------------
  function renderTable() {
    const order = [...Array(A.NV).keys()];
    const key = state.sortKey;
    const val = (i) => {
      if (key === 'name') return A.names[i];
      if (key === 'est') return A.camEst[i];
      if (key === 'obs') return A.camNObs[i];
      if (key === 'cerr') return cmpErr ? cmpErr.cerr[i] : NaN;
      if (key === 'rerr') return cmpErr ? cmpErr.rerr[i] : NaN;
      return A.camErr[i];
    };
    order.sort((a, b) => {
      const va = val(a), vb = val(b);
      if (typeof va === 'string') return va.localeCompare(vb) * state.sortDir;
      const na = Number.isFinite(va) ? va : -Infinity, nb = Number.isFinite(vb) ? vb : -Infinity;
      return (na - nb) * state.sortDir;
    });
    const rows = order
      .map((i) => {
        const selected = state.sel && state.sel.type === 'cam' && state.sel.set === 0 && state.sel.idx === i;
        const cls = [selected ? 'sel' : '', !A.camEst[i] ? 'unreg' : ''].join(' ');
        const cmpCells = cmpErr
          ? '<td class="cmp">' + fmt(cmpErr.cerr[i], 3) + '</td><td class="cmp">' + fmt(cmpErr.rerr[i], 2) + '</td>'
          : '<td class="cmp"></td><td class="cmp"></td>';
        return (
          '<tr class="' + cls + '" data-cam="' + i + '"><td>' + esc(A.names[i]) + '</td><td>' +
          (A.camEst[i] ? 'yes' : '<span class="warn">no</span>') + '</td><td>' + A.camNObs[i] + '</td><td>' +
          fmt(A.camErr[i], 2) + '</td>' + cmpCells + '</tr>'
        );
      })
      .join('');
    $('views-body').innerHTML = rows;
    $('views-body').querySelectorAll('tr').forEach((tr) =>
      tr.addEventListener('click', () => select('cam', Number(tr.dataset.cam), 0))
    );
    const unreg = A.NV - A.stats.num_estimated_views;
    let text = A.NV + ' views, ' + A.stats.num_estimated_views + ' registered' + (unreg ? ', ' + unreg + ' unregistered' : '');
    if (cmpErr) text += '; ' + CMP.num_shared + ' shared with B (errors after alignment)';
    $('views-summary').textContent = text;
  }

  // ---- legend and summary -------------------------------------------------
  function updateLegend() {
    const bar = $('legend-bar');
    const txt = $('legend-text');
    if (state.colorMode === 'rgb') {
      bar.style.display = 'none';
      txt.textContent = isCompare
        ? 'Colors from the reconstruction (RGB). B is tinted magenta.'
        : 'Colors from the reconstruction (RGB).';
      return;
    }
    bar.style.display = 'block';
    bar.style.background = 'linear-gradient(to right, ' +
      [0, 0.25, 0.5, 0.75, 1].map((t) => {
        const c = ramp(t);
        return 'rgb(' + c.map((v) => Math.round(v * 255)).join(',') + ') ' + Math.round(t * 100) + '%';
      }).join(', ') + ')';
    if (state.colorMode === 'track') {
      txt.textContent = 'Track length: ' + trackMin + ' (left) to ' + trackMax + ' views (right), log scale.';
    } else if (state.colorMode === 'depth') {
      txt.textContent = 'Distance from ' + depthLabel + ': 0 (left) to ' + fmt(depthNorm, 3) +
        ' scene units (99th percentile, right). Select a camera to measure from it.';
    } else {
      txt.textContent = 'Reprojection error: 0 (left) to ' + fmt(errNorm, 2) + ' px (99th percentile, right). Gray = no error.';
    }
  }

  function updateSummary() {
    const s = A.stats;
    let html =
      '<div>' + s.num_points.toLocaleString() + ' points' +
      (s.num_tracks_total > s.num_points ? ' (of ' + s.num_tracks_total.toLocaleString() + ', subsampled)' : '') + '</div>' +
      '<div>' + s.num_observations.toLocaleString() + ' observations</div>' +
      '<div>mean reprojection error ' + fmt(s.mean_reprojection_error, 3) + ' px</div>';
    if (isCompare) {
      const b = sets[1].stats;
      html +=
        '<div class="cmp-line">B (aligned): ' + b.num_points.toLocaleString() + ' points, ' +
        sets[1].NV + ' views, ' + b.num_estimated_views + ' registered</div>' +
        '<div class="cmp-line">Alignment: ' + CMP.num_shared + ' shared views, scale ' +
        fmt(CMP.sim3.scale, 4) + ', RMS center error ' + fmt(CMP.rms_center_error, 4) + ' scene units</div>';
    }
    $('summary').innerHTML = html;
  }

  // ---- visibility and theme -----------------------------------------------
  function applyVisibility() {
    let anyGraph = false;
    for (const s of sets) {
      const on = state.sets[s.idx];
      s.pointsObj.visible = on && state.showPoints;
      s.centObj.visible = on && state.showCams;
      s.frusObj.visible = on && state.showCams && state.showFrusta;
      if (s.graphObj) {
        anyGraph = true;
        s.graphObj.visible = on && state.showGraph;
      }
      s.trajObj.visible = on && state.showTraj && s.trajCount >= 2;
    }
    const g = $('showGraph');
    if (g) g.disabled = !anyGraph;
  }

  function applyTheme(theme) {
    document.documentElement.dataset.theme = theme;
    scene3.background = new THREE.Color(theme === 'light' ? 0xf5f6f8 : 0x0f1115);
    $('theme').value = theme;
  }

  // ---- measure ------------------------------------------------------------
  function updateMeasure() {
    const pts = state.meas;
    setAttr(measGeom, 'position', Float32Array.from(pts.reduce((acc, p) => acc.concat(p.pos), [])), 3);
    measPts.visible = pts.length > 0;
    if (pts.length === 2) {
      setAttr(measLineGeom, 'position', Float32Array.from(pts[0].pos.concat(pts[1].pos)), 3);
      measLine.visible = true;
    } else {
      setAttr(measLineGeom, 'position', new Float32Array(0), 3);
      measLine.visible = false;
    }
  }

  function measureClick(hit) {
    if (!hit) return;
    if (state.meas.length >= 2) state.meas = [];
    const s = sets[hit.set];
    const label = (hit.type === 'cam' ? 'camera ' + s.names[hit.idx] : 'point ' + fmt(s.trackIds[hit.idx], 0)) +
      (isCompare ? ' [' + PALETTE[s.idx].label + ']' : '');
    state.meas.push({ pos: hit.pos, label });
    updateMeasure();
    if (state.meas.length === 2) showMeasure();
    else clearInfo('Measure: click a second point.');
  }

  // ---- fly mode -----------------------------------------------------------
  const fly = { yaw: 0, pitch: 0, speed: 0.5 * radius, keys: new Set() };

  function lookAt(target) {
    const dx = target.x - camera.position.x, dy = target.y - camera.position.y, dz = target.z - camera.position.z;
    const len = Math.hypot(dx, dy, dz) || 1;
    fly.yaw = Math.atan2(-dx, -dz);
    fly.pitch = clamp(Math.asin(dy / len), -1.5, 1.5);
  }

  function setFly(on) {
    on = !!on;
    if (on === state.fly) return;
    state.fly = on;
    $('flyMode').checked = on;
    const dir = camera.getWorldDirection(new THREE.Vector3());
    if (on) {
      fly.yaw = Math.atan2(-dir.x, -dir.z);
      fly.pitch = clamp(Math.asin(clamp(dir.y, -1, 1)), -1.5, 1.5);
      fly.keys.clear();
      controls.enabled = false;
    } else {
      controls.enabled = true;
      controls.target.copy(camera.position).addScaledVector(dir, Math.max(0.25 * radius, 1e-6));
      controls.update();
    }
    updateHint();
  }

  function updateFly(dt) {
    const cp = Math.cos(fly.pitch), sp = Math.sin(fly.pitch);
    const sy = Math.sin(fly.yaw), cy = Math.cos(fly.yaw);
    const fwd = [-sy * cp, sp, -cy * cp];
    const right = [cy, 0, -sy];
    let mx = 0, my = 0, mz = 0;
    const add = (v, k) => { mx += v[0] * k; my += v[1] * k; mz += v[2] * k; };
    const keys = fly.keys;
    if (keys.has('KeyW')) add(fwd, 1);
    if (keys.has('KeyS')) add(fwd, -1);
    if (keys.has('KeyD')) add(right, 1);
    if (keys.has('KeyA')) add(right, -1);
    if (keys.has('Space') || keys.has('KeyE')) my += 1;
    if (keys.has('ShiftLeft') || keys.has('ShiftRight') || keys.has('KeyQ')) my -= 1;
    const len = Math.hypot(mx, my, mz);
    if (len > 0) {
      const k = (fly.speed * dt) / len;
      camera.position.x += mx * k;
      camera.position.y += my * k;
      camera.position.z += mz * k;
    }
    camera.rotation.order = 'YXZ';
    camera.rotation.set(fly.pitch, fly.yaw, 0);
  }

  function updateHint() {
    let text = state.fly
      ? 'Fly: W/A/S/D move, Space/E up, Shift/Q down, drag to look, wheel changes speed.'
      : 'Drag: orbit. Right-drag: pan. Wheel: zoom. Click a camera or point.';
    if (state.measure) text = 'Measure: click two points (distance in scene units). ' + text;
    $('hint').textContent = text;
  }

  function selectionPosition() {
    const sel = state.sel;
    if (!sel) return null;
    const s = sets[sel.set];
    if (sel.type === 'point') return Array.from(s.xyz.subarray(3 * sel.idx, 3 * sel.idx + 3));
    if (sel.type === 'cam') return Array.from(s.camC.subarray(3 * sel.idx, 3 * sel.idx + 3));
    return null;
  }

  // ---- controls -----------------------------------------------------------
  function bindControls() {
    $('colorMode').addEventListener('change', (e) => {
      state.colorMode = e.target.value;
      computeColors();
      rebuildAllPoints();
      updateLegend();
    });
    $('pointSize').addEventListener('input', (e) => {
      state.pointSize = Number(e.target.value);
      sets.forEach((s) => { s.pointMat.size = state.pointSize; });
      $('pointSizeVal').textContent = state.pointSize;
    });
    $('minTrack').addEventListener('input', (e) => {
      state.minTrack = Number(e.target.value);
      $('minTrackVal').textContent = state.minTrack;
      rebuildAllPoints();
    });
    $('maxErr').addEventListener('input', (e) => {
      state.maxErr = Number(e.target.value);
      $('maxErrVal').textContent = state.maxErr >= errMaxAll ? 'off' : fmt(state.maxErr, 2) + ' px';
      rebuildAllPoints();
    });
    const toggles = { showPoints: 'showPoints', showCams: 'showCams', showFrusta: 'showFrusta', showUnreg: 'showUnreg', showGraph: 'showGraph', showTraj: 'showTraj' };
    Object.keys(toggles).forEach((id) => {
      const el = $(id);
      if (!el) return;
      el.addEventListener('change', (e) => {
        state[toggles[id]] = e.target.checked;
        applyVisibility();
        refreshCameraColors();
      });
    });
    $('trajOrder').addEventListener('change', (e) => {
      state.trajOrder = e.target.value;
      sets.forEach(buildTraj);
      applyVisibility();
    });
    ['setA', 'setB'].forEach((id, i) => {
      const el = $(id);
      if (!el || i >= sets.length) return;
      el.addEventListener('change', (e) => {
        state.sets[i] = e.target.checked;
        rebuildAllPoints();
        applyVisibility();
        renderTable();
      });
    });
    $('flyMode').addEventListener('change', (e) => setFly(e.target.checked));
    $('measure').addEventListener('change', (e) => {
      state.measure = e.target.checked;
      updateHint();
      showDefaultInfo();
    });
    $('measure-clear').addEventListener('click', () => {
      state.meas = [];
      updateMeasure();
      showDefaultInfo();
    });
    $('theme').addEventListener('change', (e) => applyTheme(e.target.value));
    $('screenshot').addEventListener('click', screenshot);
    $('reset').addEventListener('click', () => {
      camera.position.copy(defaultPos);
      if (state.fly) {
        lookAt(center);
      } else {
        controls.target.copy(center);
        controls.update();
      }
    });
    $('center-sel').addEventListener('click', () => {
      const t = selectionPosition();
      if (!t) return;
      if (state.fly) {
        lookAt(new THREE.Vector3(t[0], t[1], t[2]));
      } else {
        controls.target.set(t[0], t[1], t[2]);
        controls.update();
      }
    });
    document.querySelectorAll('#views thead th[data-key]').forEach((th) =>
      th.addEventListener('click', () => {
        const k = th.dataset.key;
        state.sortDir = state.sortKey === k ? -state.sortDir : 1;
        state.sortKey = k;
        renderTable();
      })
    );
  }

  function screenshot() {
    renderer.render(scene3, camera);
    const a = document.createElement('a');
    a.href = canvas.toDataURL('image/png');
    a.download = 'pytheia_view.png';
    document.body.appendChild(a);
    a.click();
    a.remove();
  }

  // keyboard (fly mode) and mouse (orbit: OrbitControls; fly: drag to look)
  window.addEventListener('keydown', (e) => {
    if (!state.fly) return;
    fly.keys.add(e.code);
    if (e.code === 'Space') e.preventDefault();
  });
  window.addEventListener('keyup', (e) => fly.keys.delete(e.code));
  window.addEventListener('blur', () => fly.keys.clear());
  window.addEventListener('pointerup', () => { canvas._look = null; });
  canvas.addEventListener('wheel', (e) => {
    if (!state.fly) return;
    e.preventDefault();
    fly.speed *= e.deltaY < 0 ? 1.25 : 0.8;
  }, { passive: false });
  canvas.addEventListener('pointermove', (e) => {
    if (!state.fly || !canvas._look) return;
    const dx = e.clientX - canvas._look.x, dy = e.clientY - canvas._look.y;
    canvas._look = { x: e.clientX, y: e.clientY };
    fly.yaw -= dx * 0.004;
    fly.pitch = clamp(fly.pitch - dy * 0.004, -1.5, 1.5);
  });

  canvas.addEventListener('pointerdown', (e) => {
    canvas._down = { x: e.clientX, y: e.clientY, t: performance.now() };
    if (state.fly) canvas._look = { x: e.clientX, y: e.clientY };
  });
  canvas.addEventListener('pointerup', (e) => {
    const d = canvas._down;
    canvas._down = null;
    canvas._look = null;
    if (!d) return;
    if (Math.hypot(e.clientX - d.x, e.clientY - d.y) > 4 || performance.now() - d.t > 500) return;
    const r = canvas.getBoundingClientRect();
    const hit = pickAt(e.clientX - r.left, e.clientY - r.top);
    if (state.measure) {
      measureClick(hit);
      return;
    }
    if (hit) select(hit.type, hit.idx, hit.set);
    else select(null, -1);
  });

  // ---- sizing and loop ----------------------------------------------------
  function resize() {
    const w = Math.max(1, host.clientWidth), h = Math.max(1, host.clientHeight);
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize);
  resize();

  let lastT = performance.now();
  function animate() {
    requestAnimationFrame(animate);
    const now = performance.now();
    const dt = clamp((now - lastT) / 1000, 0, 0.1);
    lastT = now;
    if (state.fly) updateFly(dt);
    else controls.update();
    renderer.render(scene3, camera);
  }

  // ---- init ---------------------------------------------------------------
  $('colorMode').value = state.colorMode;
  $('pointSize').value = state.pointSize;
  $('pointSizeVal').textContent = state.pointSize;
  sets.forEach((s) => { s.pointMat.size = state.pointSize; });
  $('minTrack').min = 1;
  $('minTrack').max = Math.max(1, Math.round(trackMax));
  $('minTrack').value = 1;
  $('minTrackVal').textContent = 1;
  $('maxErr').min = 0;
  $('maxErr').max = Math.max(errMaxAll, 1e-6);
  $('maxErr').step = Math.max(errMaxAll, 1e-6) / 500;
  $('maxErr').value = errMaxAll;
  $('maxErrVal').textContent = 'off';
  const toggleIds = ['showPoints', 'showCams', 'showFrusta', 'showUnreg', 'showGraph', 'showTraj'];
  toggleIds.forEach((id) => {
    if ($(id)) $(id).checked = state[id];
  });
  $('trajOrder').value = state.trajOrder;
  if (isCompare) {
    $('setA').checked = true;
    $('setB').checked = true;
  } else {
    $('row-sets').style.display = 'none';
  }
  if (!sets.some((s) => s.edges)) $('row-graph').style.display = 'none';
  if (A.NV === 0) $('row-cams').style.display = 'none';
  sets.forEach(buildTraj);
  updateSummary();
  computeColors();
  rebuildAllPoints();
  refreshCameraColors();
  updateSelectionObjects();
  updateMeasure();
  updateLegend();
  renderTable();
  applyVisibility();
  applyTheme(document.documentElement.dataset.theme || 'dark');
  bindControls();
  clearInfo();
  updateHint();
  // scripting hook: window.pytheia.select('cam' | 'point', index, set = 0) or select(null)
  // viewport position (CSS px) of a camera or point, or null when behind the camera
  function screenPos(type, setIdx, idx) {
    const s = sets[setIdx];
    const arr = type === 'cam' ? s.camC : s.xyz;
    const q = project(screenMatrix(), arr[3 * idx], arr[3 * idx + 1], arr[3 * idx + 2], canvas.clientWidth, canvas.clientHeight);
    if (!q) return null;
    const r = canvas.getBoundingClientRect();
    return [r.left + q[0], r.top + q[1]];
  }
  window.pytheia = { select, state, controls, camera, sets: sets.length, setFly, lookAt, screenPos };
  animate();
}

(async () => {
  try {
    const { THREE, OrbitControls } = await loadThree();
    main(THREE, OrbitControls);
  } catch (err) {
    const hint = document.getElementById('hint');
    if (hint) hint.textContent = 'Viewer failed to start: ' + err;
    console.error(err);
  }
})();
