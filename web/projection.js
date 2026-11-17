// Robot frame: X right, Y down, Z forward. Camera bases use the same axes.
export const radians = deg => deg * Math.PI / 180;
const dot = (a, b) => a.reduce((n, v, i) => n + v * b[i], 0);

export function basis(yaw, pitch, roll = 0) {
  const y = radians(yaw), p = radians(pitch), a = radians(roll);
  const sy = Math.sin(y), cy = Math.cos(y), sp = Math.sin(p), cp = Math.cos(p);
  const r = [cy, 0, -sy], d = [-sy * sp, cp, -cy * sp], f = [sy * cp, sp, cy * cp];
  return [
    r.map((v, i) => v * Math.cos(a) + d[i] * Math.sin(a)),
    d.map((v, i) => v * Math.cos(a) - r[i] * Math.sin(a)),
    f,
  ];
}

export function cameraRay(world, camBasis) {
  return camBasis.map(axis => dot(world, axis));
}

export function worldRay(camera, camBasis) {
  return [0, 1, 2].map(i => camBasis.reduce((v, axis, j) => v + axis[i] * camera[j], 0));
}

export function projectLens(ray, cam, width, height) {
  const [x, y, z] = cameraRay(ray, basis(cam.yaw, cam.pitch, cam.roll));
  const radial = Math.hypot(x, y);
  if (radial < 1e-8 && z <= 0) return null;
  const r = cam.focal * Math.atan2(radial, z);
  const dx = radial < 1e-8 ? 0 : x * r / radial;
  const dy = radial < 1e-8 ? 0 : y * r / radial;
  let px = cam.cx + dx, py = cam.cy + dy;
  if (cam.rotate180) { px = width - px; py = height - py; }
  if (r >= cam.radius || dy >= cam.lowerCut + cam.lowerSlope * dx ||
      px < 0 || px >= width || py < 0 || py >= height) return null;
  return [px / width, py / height];
}

export function groundPoint(u, v, map) {
  return [(u * 2 - 1) * map.halfWidth, 0, (1 - v * 2) * map.halfDepth];
}

export function groundRay(world, cam) {
  return [world[0] - cam.x, cam.height, world[2] - cam.z];
}

export function previewGeometry(profile, rear, mirror, aspect) {
  const tx = Math.tan(radians(profile.hfov) / 2);
  return {
    axes: basis(rear ? 180 : 0, profile.lookDown),
    tx, ty: tx / aspect, sign: rear && mirror ? -1 : 1,
  };
}

export function previewPoint(ray, view) {
  const [x, y, z] = cameraRay(ray, view.axes);
  if (z <= 0) return null;
  return [(1 + view.sign * x / (z * view.tx)) / 2, (1 + y / (z * view.ty)) / 2];
}

export const FS = `precision highp float;
varying vec2 uv;
uniform sampler2D frontImage, rearImage;
uniform mat3 frontBasis, rearBasis, viewToWorld;
uniform vec4 frontLens, rearLens; // cx, cy, usable radius, focal (pixels)
uniform vec3 frontCut, rearCut;   // lower pixel cutoff, slope, rotate180
uniform vec3 frontPos, rearPos;   // robot XYZ with down positive
uniform vec2 textureSize, extent, tanFov, online;
uniform float mode, rearView, mirrorSign, allowBlend;

// xy = sensor UV, z = coverage/quality. Zero means never sample.
vec3 lens(vec3 ray, vec4 p, vec3 cut) {
  float radial = length(ray.xy);
  if (radial < 1e-6 && ray.z <= 0.0) return vec3(0.0);
  float r = p.w * atan(radial, ray.z);
  vec2 off = (r / max(radial, 1e-6)) * ray.xy;
  vec2 px = p.xy + off;
  if (cut.z > 0.5) px = textureSize - px;
  float edge = min(min(px.x, textureSize.x - px.x), min(px.y, textureSize.y - px.y));
  edge = min(edge, min(p.z - r, cut.x + cut.y * off.x - off.y));
  float q = smoothstep(0.0, 7.0, edge) * (1.0 - 0.72 * clamp(r / p.z, 0.0, 1.0));
  return vec3(px / textureSize, q);
}
vec3 hatch() {
  return mix(vec3(0.071, 0.112, 0.166), vec3(0.137, 0.191, 0.267),
             step(0.72, fract((gl_FragCoord.x + gl_FragCoord.y) / 13.0)));
}
void main() {
  vec3 missing = hatch();
  if (mode > 1.5) {
    vec3 c = missing;
    if (rearView > 0.5) {
      if (online.y > 0.5) c = texture2D(rearImage, uv).rgb;
    } else if (online.x > 0.5) c = texture2D(frontImage, uv).rgb;
    gl_FragColor = vec4(c, 1.0);
    return;
  }
  if (mode > 0.5) {
    vec3 ray = viewToWorld * vec3((2.0 * uv.x - 1.0) * tanFov.x * mirrorSign,
                                 (2.0 * uv.y - 1.0) * tanFov.y, 1.0);
    vec3 c = missing;
    if (rearView > 0.5) {
      vec3 s = lens(rearBasis * ray, rearLens, rearCut);
      if (online.y > 0.5 && s.z > 0.0) c = mix(missing, texture2D(rearImage, s.xy).rgb, smoothstep(0.0, 0.24, s.z));
    } else {
      vec3 s = lens(frontBasis * ray, frontLens, frontCut);
      if (online.x > 0.5 && s.z > 0.0) c = mix(missing, texture2D(frontImage, s.xy).rgb, smoothstep(0.0, 0.24, s.z));
    }
    gl_FragColor = vec4(c, 1.0);
    return;
  }
  // Both cameras see the *same* world point, including across the sides.
  vec3 point = vec3((2.0 * uv.x - 1.0) * extent.x, 0.0, (1.0 - 2.0 * uv.y) * extent.y);
  vec3 a = lens(frontBasis * (point - frontPos), frontLens, frontCut);
  vec3 b = lens(rearBasis * (point - rearPos), rearLens, rearCut);
  a.z *= online.x; b.z *= online.y;
  vec3 c = missing;
  if (a.z > 0.0 && b.z > 0.0) {
    if (allowBlend > 0.5) {
      float w = smoothstep(0.22, 0.78, a.z / (a.z + b.z));
      c = mix(texture2D(rearImage, b.xy).rgb, texture2D(frontImage, a.xy).rgb, w);
    } else if (a.z > b.z) c = texture2D(frontImage, a.xy).rgb;
    else c = texture2D(rearImage, b.xy).rgb;
  } else if (a.z > 0.0) c = texture2D(frontImage, a.xy).rgb;
  else if (b.z > 0.0) c = texture2D(rearImage, b.xy).rgb;
  gl_FragColor = vec4(mix(missing, c, smoothstep(0.0, 0.20, max(a.z, b.z))), 1.0);
}`;
