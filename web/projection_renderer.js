import { basis, FS } from './projection.js?v=347-sync-1';
const cameras = ['front', 'rear'];
const vs = `
attribute vec2 position;
varying vec2 uv;
void main() {
  uv = (position * vec2(1.0, -1.0) + 1.0) * 0.5;
  gl_Position = vec4(position, 0.0, 1.0);
}`;

export function createRenderer(canvas, config, preserveDrawingBuffer = false) {
  const gl = canvas.getContext('webgl', { antialias: false, depth: false, preserveDrawingBuffer });
  if (!gl) throw new Error('浏览器未启用 WebGL');
  function compile(type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader));
    return shader;
  }
  const program = gl.createProgram();
  const vertex = compile(gl.VERTEX_SHADER, vs), fragment = compile(gl.FRAGMENT_SHADER, FS);
  gl.attachShader(program, vertex);
  gl.attachShader(program, fragment);
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
  gl.deleteShader(vertex);
  gl.deleteShader(fragment);
  gl.useProgram(program);
  gl.bindBuffer(gl.ARRAY_BUFFER, gl.createBuffer());
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1,-1, 1,-1, -1,1, 1,1]), gl.STATIC_DRAW);
  const a = gl.getAttribLocation(program, 'position');
  gl.enableVertexAttribArray(a);
  gl.vertexAttribPointer(a, 2, gl.FLOAT, false, 0, 0);
  const U = Object.fromEntries(`frontImage rearImage frontBasis rearBasis viewToWorld
    frontLens rearLens frontCut rearCut frontPos rearPos textureSize extent tanFov online
    mode rearView mirrorSign allowBlend`.trim().split(/\s+/).map(n => [n, gl.getUniformLocation(program, n)]));
  const textures = {};
  const {width, height} = config;
  for (const [unit, name] of cameras.entries()) {
    textures[name] = gl.createTexture();
    gl.activeTexture(gl.TEXTURE0 + unit);
    gl.bindTexture(gl.TEXTURE_2D, textures[name]);
    for (const n of [gl.TEXTURE_MIN_FILTER, gl.TEXTURE_MAG_FILTER]) gl.texParameteri(gl.TEXTURE_2D, n, gl.LINEAR);
    for (const n of [gl.TEXTURE_WRAP_S, gl.TEXTURE_WRAP_T]) gl.texParameteri(gl.TEXTURE_2D, n, gl.CLAMP_TO_EDGE);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, width, height, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
    gl.uniform1i(U[name + 'Image'], unit);
  }
  gl.uniform2f(U.textureSize, width, height);
  return {
    upload(name, bitmap) {
      if (bitmap.width !== width || bitmap.height !== height) throw new Error('鱼眼分辨率与配置不符');
      gl.activeTexture(gl.TEXTURE0 + cameras.indexOf(name));
      gl.bindTexture(gl.TEXTURE_2D, textures[name]);
      gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, gl.RGBA, gl.UNSIGNED_BYTE, bitmap);
    },
    begin(profile, live, blend) {
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT);
      gl.uniform2f(U.online, Number(live.front), Number(live.rear));
      gl.uniform2f(U.extent, profile.map.halfWidth, profile.map.halfDepth);
      gl.uniform1f(U.allowBlend, Number(blend));
      for (const name of cameras) {
        const c = profile[name];
        const axes = basis(c.yaw, c.pitch, c.roll);
        gl.uniformMatrix3fv(U[name + 'Basis'], false, [0, 1, 2].flatMap(i => axes.map(a => a[i])));
        gl.uniform4f(U[name + 'Lens'], c.cx, c.cy, c.radius, c.focal);
        gl.uniform3f(U[name + 'Cut'], c.lowerCut, c.lowerSlope, Number(c.rotate180));
        gl.uniform3f(U[name + 'Pos'], c.x, -c.height, c.z);
      }
    },
    draw(rect, origin, mode, rear, view) {
      if (!rect.width || !rect.height) return;
      const sx = canvas.width / origin.width, sy = canvas.height / origin.height;
      gl.viewport(Math.round((rect.left-origin.left)*sx), Math.round((origin.bottom-rect.bottom)*sy),
        Math.round(rect.width*sx), Math.round(rect.height*sy));
      gl.uniform1f(U.mode, mode);
      gl.uniform1f(U.rearView, Number(rear));
      if (view) {
        gl.uniformMatrix3fv(U.viewToWorld, false, view.axes.flat());
        gl.uniform2f(U.tanFov, view.tx, view.ty);
        gl.uniform1f(U.mirrorSign, view.sign);
      }
      gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    }
  };
}

