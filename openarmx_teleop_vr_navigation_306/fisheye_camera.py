"""Demand-driven native JPEG forwarding; the vision service owns the camera."""

from collections import deque
import struct
import threading
import time


def jpeg_size(data):
    """Read JPEG SOF geometry without decoding any image pixels."""
    if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
        raise ValueError('incomplete JPEG')
    offset = 2
    while offset + 4 <= len(data):
        if data[offset] != 0xff:
            raise ValueError('invalid JPEG marker')
        while offset < len(data) and data[offset] == 0xff:
            offset += 1
        marker = data[offset]
        offset += 1
        if marker in (0xd8, 0xd9, 0x01) or 0xd0 <= marker <= 0xd7:
            continue
        length = int.from_bytes(data[offset:offset + 2], 'big')
        if length < 2 or offset + length > len(data):
            raise ValueError('invalid JPEG segment')
        if marker in (0xc0, 0xc1, 0xc2, 0xc3, 0xc5, 0xc6, 0xc7,
                      0xc9, 0xca, 0xcb, 0xcd, 0xce, 0xcf):
            if length < 8:
                raise ValueError('invalid JPEG SOF')
            height, width = struct.unpack_from('>HH', data, offset + 3)
            return width, height
        if marker == 0xda:
            break
        offset += length
    raise ValueError('JPEG geometry missing')


class FisheyeCamera:
    """One shared latest-frame reader, opened only while clients subscribe."""

    def __init__(self, *, module_name='mod_camera_head_right', image_key='head_right',
                 label='鱼眼辅助', consumer_name='vr345_fisheye_jpeg',
                 maximum_fps=10.0, consumer_factory=None):
        self._module_name = str(module_name)
        self._image_key = str(image_key)
        self._label = str(label)
        self._consumer_name = str(consumer_name)
        self.maximum_fps = max(1.0, min(10.0, float(maximum_fps)))
        self._factory = consumer_factory
        self._condition = threading.Condition(threading.RLock())
        self._lifecycle = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._subscribers = 0
        self._closed = False
        self._jpeg = None
        self._version = 0
        self._published_ns = 0
        self._width, self._height = 640, 360
        self._received_at = 0.0
        self._times = deque(maxlen=30)
        self._error = ''
        self._forwarded = 0

    def acquire(self):
        with self._lifecycle:
            with self._condition:
                if self._closed:
                    raise RuntimeError('fisheye reader closed')
                if self._subscribers >= 2:
                    raise RuntimeError('fisheye client limit reached')
                self._subscribers += 1
                if self._subscribers != 1:
                    return
                self._jpeg = None
                self._published_ns = 0
                self._received_at = 0.0
                self._times.clear()
                self._error = ''
                self._stop.clear()
            self._thread = threading.Thread(target=self._run,
                                           name='vr-fisheye-jpeg', daemon=True)
            self._thread.start()

    def release(self):
        with self._lifecycle:
            with self._condition:
                self._subscribers = max(0, self._subscribers - 1)
                if self._subscribers:
                    return
                self._stop.set()
                self._jpeg = None
                self._condition.notify_all()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
                if self._thread.is_alive():
                    raise RuntimeError('fisheye reader did not stop')
                self._thread = None

    def close(self):
        with self._condition:
            self._closed = True
            self._subscribers = 1
        self.release()

    def status(self):
        with self._condition:
            wall_age = ((time.time_ns() - self._published_ns) / 1e9
                        if self._published_ns else None)
            mono_age = (time.monotonic() - self._received_at
                        if self._received_at else None)
            age = max(wall_age, mono_age) if wall_age is not None else None
            online = bool(self._subscribers and self._jpeg and not self._error
                          and age is not None and 0 <= wall_age < 1 and age < 1)
            times = tuple(self._times)
            fps = ((len(times) - 1) / (times[-1] - times[0])
                   if online and len(times) > 1 and times[-1] > times[0] else 0)
            return {'image_key': self._image_key, 'name': self._label,
                    'online': online, 'width': self._width, 'height': self._height,
                    'fps': round(fps, 2), 'target_fps': self.maximum_fps,
                    'frame_version': self._version,
                    'frame_age': None if age is None else round(age, 3),
                    'publish_epoch_ns': self._published_ns,
                    'subscribers': self._subscribers,
                    'reader_active': bool(self._thread and self._thread.is_alive()),
                    'forwarded_frames': self._forwarded,
                    'transport': 'native_jpeg_render_ack_latest_v1',
                    'last_error': self._error}

    def snapshot(self, after_frame_id=-1, timeout=0.4):
        deadline = time.monotonic() + max(0, timeout)
        with self._condition:
            while (not self._stop.is_set()
                   and not (self._version > after_frame_id and self.status()['online'])
                   and time.monotonic() < deadline):
                self._condition.wait(max(0, deadline - time.monotonic()))
            status = self.status()
            jpeg = self._jpeg if status['online'] and self._version > after_frame_id else None
            return jpeg, self._version, status

    def _open_consumer(self):
        if self._factory is not None:
            return self._factory()
        try:
            from autolife_robot_sdk.utils import list_camera_shm_outputs, open_camera_shm_consumer
        except ImportError:
            from autolife_v3_camera import list_camera_shm_outputs, open_camera_shm_consumer
        matches = [o for o in list_camera_shm_outputs(
            'autolife_s1', 'robot_v3_1', module_name=self._module_name)
                   if str(o.output_name).upper() == 'JPEG']
        if len(matches) != 1:
            raise RuntimeError('expected one JPEG SHM output for ' + self._module_name)
        return open_camera_shm_consumer(matches[0], name=self._consumer_name)

    def _run(self):
        consumer = None
        interval = 1 / self.maximum_fps
        last_key = None
        last_received = time.monotonic()
        next_emit = 0.0
        try:
            while not self._stop.is_set():
                try:
                    if consumer is None:
                        consumer = self._open_consumer()
                        consumer.reset_frame_position()
                        last_key = None
                        last_received = time.monotonic()
                    item = consumer.get_latest(nonblock=False, timeout=0.25, with_meta=True)
                    now = time.monotonic()
                    if item is None:
                        if now - last_received > 1.0:
                            raise RuntimeError('no fresh fish-eye frames; reopening SHM')
                        continue
                    raw, frame_id, meta = item
                    published = int(meta.get('publish_epoch_ns', 0))
                    key = (int(frame_id), published)
                    age = (time.time_ns() - published) / 1e9
                    if published <= 0 or not 0 <= age < 1:
                        raise ValueError('stale or missing source timestamp')
                    if key == last_key:
                        continue
                    last_key = key
                    last_received = now
                    if now + 0.002 < next_emit:
                        continue
                    dimensions = (int(meta.get('width', 0)), int(meta.get('height', 0)))
                    if min(dimensions) <= 0:
                        raise ValueError('fisheye source geometry missing')
                    if str(meta.get('pixel_format_str', '')).upper() not in ('MJPG', 'JPEG'):
                        raise ValueError('fisheye source must be JPEG')
                    jpeg = bytes(raw)
                    if not 128 <= len(jpeg) <= 4 * 1024 * 1024 or jpeg_size(jpeg) != dimensions:
                        raise ValueError('invalid native JPEG payload')
                    next_emit = next_emit + interval if next_emit else now + interval
                    if next_emit < now - interval:
                        next_emit = now + interval
                    with self._condition:
                        self._jpeg = jpeg
                        self._width, self._height = dimensions
                        self._version += 1
                        self._published_ns = published
                        self._received_at = now
                        self._times.append(now)
                        self._error = ''
                        self._forwarded += 1
                        self._condition.notify_all()
                except Exception as error:
                    with self._condition:
                        self._jpeg = None
                        self._error = str(error)[:160]
                        self._condition.notify_all()
                    if consumer is not None:
                        consumer.close()
                        consumer = None
                    self._stop.wait(0.5)
        finally:
            if consumer is not None:
                consumer.close()


class FisheyeEndpoint:
    """Bounded binary JPEG websocket with one acknowledged frame in flight."""

    def __init__(self, camera):
        self.camera = camera
        self.sockets = set()

    async def status(self, request):
        from aiohttp import web
        return web.json_response(self.camera.status(), headers={'Cache-Control': 'no-store'})

    async def websocket(self, request):
        import asyncio
        from aiohttp import WSMsgType, web
        socket = web.WebSocketResponse(heartbeat=10, max_msg_size=4096,
                                       compress=False)
        acquired = False
        try:
            await asyncio.to_thread(self.camera.acquire)
            acquired = True
            await socket.prepare(request)
            self.sockets.add(socket)
            frame_id = -1
            while not socket.closed:
                jpeg, version, status = await asyncio.to_thread(
                    self.camera.snapshot, frame_id, 0.4)
                await asyncio.wait_for(socket.send_json(
                    {'type': 'fisheye_status', 'status': status}), timeout=2)
                if jpeg is None:
                    continue
                await asyncio.wait_for(socket.send_bytes(jpeg), timeout=2)
                # Slow clients cannot build an unbounded TCP image backlog.
                message = await socket.receive(timeout=3)
                if message.type != WSMsgType.TEXT:
                    break
                ack = message.json()
                if (not isinstance(ack, dict) or ack.get('type') != 'fisheye_ack'
                        or ack.get('frame_version') != version):
                    break
                frame_id = version
        except (ConnectionError, RuntimeError, ValueError, asyncio.TimeoutError):
            pass
        finally:
            self.sockets.discard(socket)
            if acquired:
                await asyncio.to_thread(self.camera.release)
            if socket.prepared:
                await socket.close()
        if not socket.prepared:
            return web.json_response({'error': 'fisheye unavailable'}, status=503)
        return socket

    async def close(self):
        import asyncio
        await asyncio.gather(*(socket.close() for socket in tuple(self.sockets)),
                             return_exceptions=True)
