export const FRESH_MS = 850;

async function decode(blob) {
  if (typeof createImageBitmap === 'function') {
    try { return await createImageBitmap(blob); } catch { /* use the HTML decoder */ }
  }
  const url = URL.createObjectURL(blob);
  try {
    const image = new Image();
    image.src = url;
    await image.decode();
    return image;
  } finally {
    URL.revokeObjectURL(url);
  }
}

// Exactly one JPEG is in flight. Do not keep decoding or ACK an old
// generation after switching modes, leaving XR, or hiding the page.
export function createStream(path, onFrame, invalidate) {
  let ws, metadata, lastAt = -Infinity, epoch = 0, retryAt = 0, openedAt = 0;
  let samples = [], frames = 0, decoding = false;
  function clear() {
    lastAt = -Infinity;
    epoch = 0;
    samples = [];
    invalidate();
  }
  function stop() {
    const old = ws;
    ws = null;
    metadata = null;
    decoding = false;
    if (old) {
      old.onopen = old.onclose = old.onerror = old.onmessage = null;
      if (old.readyState < WebSocket.CLOSING) old.close();
      clear();
    }
  }
  function connect(now) {
    if (ws?.readyState === WebSocket.CONNECTING && now - openedAt > 3500) {
      stop(); retryAt = now + 950;
    }
    if (ws || now < retryAt) return;
    let s;
    try {
      s = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}${path}`);
    } catch {
      retryAt = now + 950; return;
    }
    ws = s;
    metadata = null;
    openedAt = now;
    s.binaryType = 'arraybuffer';
    const fail = () => {
      if (s !== ws) return;
      stop();
      retryAt = performance.now() + 950;
    };
    s.onclose = s.onerror = fail;
    s.onmessage = async e => {
      if (s !== ws) return;
      if (typeof e.data === 'string') {
        try {
          const msg = JSON.parse(e.data);
          if (msg.type === 'fisheye_status') {
            metadata = msg.status;
            if (!metadata?.online && Number.isFinite(lastAt)) clear();
          }
        } catch { fail(); }
        return;
      }
      const meta = metadata;
      if (decoding || !meta?.online || !Number.isSafeInteger(meta.frame_version) ||
          !Number.isFinite(meta.frame_age) || meta.frame_age < 0 || meta.frame_age >= 1 ||
          !(e.data instanceof ArrayBuffer) || e.data.byteLength < 128 || e.data.byteLength > 512*1024) {
        fail(); return;
      }
      decoding = true;
      const received = performance.now();
      let bitmap;
      try {
        bitmap = await decode(new Blob([e.data], {type:'image/jpeg'}));
        if (s !== ws || s.readyState !== WebSocket.OPEN) return;
        const at = performance.now();
        if (bitmap.width !== 640 || bitmap.height !== 360 || at - received > 850) {
          fail(); return;
        }
        onFrame(bitmap, meta);
        if (at-lastAt > FRESH_MS) samples = [];
        lastAt = at;
        epoch = Number(meta.publish_epoch_ns || 0)/1e6;
        frames++;
        samples.push(at);
        samples = samples.filter(t => at-t < 2100);
        s.send(JSON.stringify({type:'fisheye_ack', frame_version:meta.frame_version}));
        invalidate();
      } catch {
        fail();
      } finally {
        bitmap?.close?.();
        if (s === ws) decoding = false;
      }
    };
  }
  return {
    connect, stop,
    status(now) {
      const fresh = now-lastAt < FRESH_MS;
      const fps = fresh && samples.length > 1
        ? 1000*(samples.length-1)/Math.max(1, samples.at(-1)-samples[0]) : 0;
      return {fresh, fps, epoch, frames};
    },
  };
}
