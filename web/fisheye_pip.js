/* 347 navigation PiP: one plane, with two mutually exclusive camera modes. */
import { createRenderer } from './projection_renderer.js?v=347-sync-1';
import { createStream } from './jpeg_stream.js?v=347-sync-1';
import { groundRay, previewGeometry, previewPoint, projectLens } from './projection.js?v=347-sync-1';
import { robotBounds, robotUrl } from './surround_robot.js?v=347-sync-1';

const MODES = ['off', 'surround', 'front'];
const labels = {off:'关', surround:'后视＋俯视', front:'仅前鱼眼'};
const PANEL_W = 2.3, PANEL_H = 1.725, BOTTOM = 0.04;
const CROP = {x:150, y:0, w:400, h:360};
const SIZE = {front:[400,360], surround:[588,278]};
const REAR = {left:8, top:30, width:384, height:240};
const MAP = {left:400, top:30, width:180, height:240};
const cameraNames = ['front', 'rear'];

function layout(mode) {
  const [pw, ph] = SIZE[mode];
  const w = mode === 'surround' ? .50 : .30;
  const h = PANEL_W * w * ph / pw / PANEL_H;
  return {x:(1-w)/2, y:1-BOTTOM-h, w, h};
}

function rect(v) {
  return {...v, bottom:v.top+v.height};
}

let config, context = {}, mode = 'off', active = false;
let panel, canvas, texture, material, mesh, position = '', shape = 'off';
let frontCrop, surround, glCanvas, renderer, sprite, renderError = '';
let streams = {}, pumpTimer, paintTimer, lastDraw = -Infinity, statusKey = '';

function bindTexture() {
  mesh = panel?.getObject3D('mesh');
  if (!mesh || !material) return;
  // A-Frame can rebuild the mesh or replace its material after object3dset.
  if (mesh.material !== material) mesh.material = material;
  mesh.renderOrder = 25; // Below distance markers and the navigation menu.
}

function ensurePanel() {
  const parent = context.parent;
  if (!parent || !globalThis.THREE) return false;
  if (!panel) {
    panel = document.createElement('a-plane');
    panel.id = 'navigation-fisheye-pip';
    panel.setAttribute('material', 'shader: flat; color: #07121b; transparent: false; side: double; depthTest: false; depthWrite: false');
    panel.setAttribute('visible', false);
    panel.addEventListener('object3dset', bindTexture);
    panel.addEventListener('loaded', bindTexture);
    material = new THREE.MeshBasicMaterial({transparent:false, depthTest:false,
      depthWrite:false, side:THREE.DoubleSide, toneMapped:false});
  }
  const box = layout(mode);
  if (shape !== mode) {
    const [w,h] = SIZE[mode];
    canvas = document.createElement('canvas');
    canvas.width = w; canvas.height = h;
    texture?.dispose();
    texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = texture.magFilter = THREE.LinearFilter;
    texture.generateMipmaps = false;
    if (THREE.SRGBColorSpace) texture.colorSpace = THREE.SRGBColorSpace;
    else if (THREE.sRGBEncoding) texture.encoding = THREE.sRGBEncoding;
    material.map = texture;
    material.needsUpdate = true;
    panel.setAttribute('width', PANEL_W*box.w);
    panel.setAttribute('height', PANEL_H*box.h);
    shape = mode;
  }
  if (panel.parentNode !== parent) parent.appendChild(panel);
  const y = Number(context.verticalOffset || 0) + PANEL_H*(.5-box.y-box.h/2);
  const next = `0 ${y} -1.992`;
  if (position !== next) { panel.setAttribute('position', next); position = next; }
  bindTexture();
  return true;
}

function ensureRenderer() {
  if (renderer) return true;
  if (!config?.surround_profile) { renderError = '缺少鱼眼标定参数'; return false; }
  if (!glCanvas) {
    glCanvas = document.createElement('canvas');
    [glCanvas.width, glCanvas.height] = SIZE.surround;
    glCanvas.addEventListener('webglcontextlost', event => {
      event.preventDefault();
      renderer = null;
      if (mode === 'surround') {
        renderError = '俯视图形已中断';
        cameraNames.forEach(name => streams[name]?.stop());
        schedulePaint();
      }
    });
    glCanvas.addEventListener('webglcontextrestored', () => {
      renderError = '';
      if (active && mode === 'surround') pump();
    });
  }
  try {
    renderer = createRenderer(glCanvas, {width:640,height:360}, true);
    if (!sprite) {
      sprite = new Image();
      sprite.onload = schedulePaint;
      sprite.src = robotUrl;
    }
    return true;
  } catch {
    renderError = '头显暂无法渲染俯视';
    return false;
  }
}

function ensureFront() {
  if (frontCrop) return;
  frontCrop = document.createElement('canvas');
  frontCrop.width = CROP.w; frontCrop.height = CROP.h;
}

function onFrame(name, bitmap) {
  if (mode === 'front' && name === 'front') {
    const ctx = frontCrop.getContext('2d');
    ctx.drawImage(bitmap, CROP.x, CROP.y, CROP.w, CROP.h, 0, 0, CROP.w, CROP.h);
  } else if (mode === 'surround') renderer?.upload(name, bitmap);
}

function statuses(now) {
  return Object.fromEntries(cameraNames.map(name => [name, streams[name]?.status(now) || {fresh:false, fps:0, epoch:0, frames:0}]));
}

function text(ctx, content, x, y, color='#e4f5ff', align='left', size=17) {
  ctx.font = `bold ${size}px sans-serif`;
  ctx.fillStyle = color;
  ctx.textAlign = align;
  ctx.fillText(content,x,y);
}

function guides(ctx, p, view) {
  const rails = [];
  for (const [distance, color] of [[.5,'#ff7474'],[1,'#ffd577'],[2,'#90f0b1']]) {
    const ends = [-1,1].map(side => {
      const ray = groundRay([side*(p.map.bodyHalfWidth+.035), 0, -p.map.bodyHalfDepth-distance], p.rear);
      if (!projectLens(ray,p.rear,640,360)) return null;
      const point = previewPoint(ray,view);
      return point?.every(v => v > .02 && v < .98)
        ? [REAR.left+point[0]*REAR.width, REAR.top+point[1]*REAR.height] : null;
    });
    if (!ends.every(Boolean)) { rails.length = 0; continue; }
    const line = (a,b) => {ctx.moveTo(...a); ctx.lineTo(...b);};
    ctx.beginPath();
    line(ends[0],ends[1]);
    if (rails.length) {line(rails[0],ends[0]); line(rails[1],ends[1]);}
    ctx.strokeStyle = '#162028'; ctx.lineWidth = 3.5; ctx.stroke();
    ctx.strokeStyle = color; ctx.lineWidth = 1.6; ctx.stroke();
    rails.splice(0,2,...ends);
  }
}

function drawRobot(ctx,p) {
  const cx = MAP.left+MAP.width/2, cy = MAP.top+MAP.height/2;
  const scale = MAP.width/(2*p.map.halfWidth);
  if (sprite?.complete && sprite.naturalWidth) {
    const [x,y,w,h] = robotBounds;
    ctx.drawImage(sprite,cx+x*scale,cy+y*scale,w*scale,h*scale);
  } else {
    const w = p.map.bodyHalfWidth*scale, h = p.map.bodyHalfDepth*scale;
    ctx.fillStyle='#374b61'; ctx.fillRect(cx-w,cy-h,w*2,h*2);
  }
  ctx.strokeStyle = '#101b28'; ctx.lineWidth=3;
  ctx.font='bold 13px sans-serif'; ctx.textAlign='center';
  ctx.strokeText('前 ↑',cx,MAP.top+17);
  text(ctx,'前 ↑',cx,MAP.top+17,'#eef7ff','center',13);
}

function paint() {
  paintTimer = null;
  if (!active || !canvas || !texture) return;
  const now = performance.now(), {front,rear} = statuses(now);
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.fillStyle = '#07121b'; ctx.fillRect(0,0,w,h);
  if (mode === 'front') {
    if (front.fresh) ctx.drawImage(frontCrop,0,0);
    else text(ctx,'等待前鱼眼…',w/2,h/2,'#ffcf70','center',21);
    ctx.fillStyle = 'rgba(3,14,21,.80)'; ctx.fillRect(0,0,w,30);
    text(ctx,`前${front.fresh ? `  ${front.fps.toFixed(0)} FPS` : ''}`,10,21,front.fresh ? '#72f0b1' : '#ffcf70');
  } else {
    const p = config?.surround_profile;
    if (renderer && p && !renderError) {
      const view = previewGeometry(p.preview,true,p.preview.mirrorRear,REAR.width/REAR.height);
      renderer.begin(p,{front:front.fresh,rear:rear.fresh},
        front.fresh && rear.fresh && Math.abs(front.epoch-rear.epoch)<135);
      const origin = rect({left:0, top:0, width:w, height:h});
      renderer.draw(rect(REAR),origin,1,true,view);
      renderer.draw(rect(MAP),origin,0,false);
      ctx.drawImage(glCanvas,0,0);
      drawRobot(ctx,p);
      if (rear.fresh) guides(ctx,p,view);
      else text(ctx,'等待后鱼眼…',REAR.left+REAR.width/2,REAR.top+REAR.height/2,'#ffcf70','center');
      const mapLabel = front.fresh && rear.fresh ? '俯视' : '俯视 · 画面不全';
      text(ctx,'后视',REAR.left+2,21);
      text(ctx,mapLabel,MAP.left+2,21,front.fresh && rear.fresh ? '#e4f5ff' : '#ffcf70');
    } else text(ctx,renderError || '正在准备俯视…',w/2,h/2,'#ffcf70','center',21);
  }
  ctx.strokeStyle = '#68cbd9'; ctx.lineWidth = 3;
  ctx.strokeRect(1.5,1.5,w-3,h-3);
  texture.needsUpdate = true;
  lastDraw = now;
}

function schedulePaint() {
  if (!active || paintTimer != null) return;
  paintTimer = setTimeout(paint, Math.max(0, 50-(performance.now()-lastDraw)));
}

function pump() {
  if (!active) return;
  if (mode === 'surround' && (renderError || !ensureRenderer())) {
    schedulePaint(); return;
  }
  if (mode === 'front') ensureFront();
  const now = performance.now();
  for (const name of mode === 'surround' ? cameraNames : ['front']) streams[name]?.connect(now);
  const {front,rear} = statuses(now);
  const key = `${mode}/${front.fresh}/${rear.fresh}`;
  if (key !== statusKey) {statusKey = key; schedulePaint();}
}

function pause() {
  active = false;
  if (pumpTimer != null) {clearInterval(pumpTimer); pumpTimer = null;}
  if (paintTimer != null) {clearTimeout(paintTimer); paintTimer = null;}
  cameraNames.forEach(name => streams[name]?.stop());
  panel?.setAttribute('visible', false);
  statusKey = '';
}

function sync(next) {
  context = next || {};
  if (mode === 'off' || !context.visible || context.menuVisible || document.hidden) {
    if (active) pause();
    return;
  }
  if (!ensurePanel()) return;
  if (!active) {
    active = true;
    panel.setAttribute('visible', true);
    // Never expose a texture from a previous session while waiting for JPEGs.
    paint();
    pump();
    pumpTimer = setInterval(pump, 200);
  }
}

const facade = {
  configure(newConfig) {
    pause();
    config = newConfig || {};
    streams = Object.fromEntries(cameraNames.map(name => [
      name, createStream(config[name]?.websocket_path || `/ws/fisheye-${name}`,
        bmp => onFrame(name,bmp), schedulePaint)
    ]));
    sync(context);
  },
  get mode() {return mode;},
  get label() {return labels[mode];},
  get enabled() {return mode !== 'off';},
  setMode(next) {
    if (!MODES.includes(next) || mode === next) return;
    pause();
    mode = next;
    renderError = '';
    sync(context);
  },
  cycleMode() {
    facade.setMode(MODES[(MODES.indexOf(mode)+1)%MODES.length]);
    return labels[mode];
  },
  sync,
  getExclusionRects(w,h) {
    if (!active) return [];
    const r = layout(mode);
    return [{x:r.x*w, y:r.y*h, w:r.w*w, h:r.h*h}];
  },
  status() {
    return {mode, active, front:statuses(performance.now()).front, rear:statuses(performance.now()).rear,
      calibration_source:config?.calibration_source, texture_bound:!!texture && mesh?.material?.map===texture,
      error:renderError};
  },
};
globalThis.autolifeFisheyePip = facade;
document.addEventListener('visibilitychange', () => sync(context));
window.addEventListener('pagehide', pause);
