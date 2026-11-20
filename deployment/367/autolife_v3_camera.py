"""旧应用相机接口适配 ALSHM001；只读已有相机，不启动或占用设备。"""
from dataclasses import dataclass, replace
import json
import os
from pathlib import Path
import time


@dataclass(frozen=True)
class CameraOutput:
    output_name: str
    channel_name: str
    source_id: str
    sensor_id: int
    max_mapping_bytes: int
    width: int = 0
    height: int = 0


def list_camera_shm_outputs(robot_model, robot_version, *, module_name):
    # 使用本机配置，不沿用源机器相机编号、内参或共享内存名称。
    root = Path(os.environ.get('AUTOLIFE_CONFIG_ROOT', '/home/ubuntu/autolife_config_v3'))
    config = json.loads((root / 'sdk' / 'configs' / (robot_version + '.json')).read_text())
    modules = config[robot_version]['mod_camera_']
    settings = modules[module_name]['settings']['camera_shm']
    if not settings.get('enabled', False) or settings.get('protocol') != 'ALSHM001':
        raise RuntimeError('本机未配置 ALSHM001 相机输出')
    outputs = []
    for channel in settings['channels']:
        for plane in channel['planes']:
            outputs.append(CameraOutput(
                plane['output'], channel['name'], channel['source_id'],
                int(plane['sensor_id']), int(channel['max_mapping_bytes'])))
    return outputs


class CameraConsumer:
    def __init__(self, output):
        from autolife_robot_sdk.shm.camera import CameraReader
        from autolife_robot_sdk.shm.transport import WouldBlockError
        self.output = output
        self.reader = CameraReader.open(output.channel_name, max_mapping_bytes=output.max_mapping_bytes)
        self.sequence = 0
        self.last_key = None
        try:
            deadline = time.monotonic() + 1.0
            while True:
                try:
                    if str(self.reader.header.source_id) != output.source_id:
                        raise RuntimeError('相机共享内存来源与本机配置不一致')
                    self.descriptor = self.reader.descriptor
                    break
                except WouldBlockError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.005)
        except BaseException:
            self.reader.close()
            raise

    def get_latest(self, *, nonblock=False, timeout=.5, with_meta=False):
        from autolife_robot_sdk.shm.camera import PixelFormat
        from autolife_robot_sdk.shm.transport import WouldBlockError
        try:
            result = self.reader.read(mode='latest', timeout=0 if nonblock else max(0., timeout))
        except WouldBlockError:
            return None
        if result.status.value in ('closed', 'owner_lost'):
            raise RuntimeError('相机生产进程已退出')
        frame = result.frame
        if frame is None:
            return None
        key = (str(frame.raw.generation), frame.raw.stream_epoch, frame.raw.slot.sequence)
        if key == self.last_key:
            return None
        planes = [p for p in frame.planes if p.sensor_id == self.output.sensor_id]
        if len(planes) != 1:
            raise RuntimeError('相机帧中找不到唯一的指定图像平面')
        plane = planes[0]
        # 独立拷贝，避免异步编码时生产者覆盖缓冲区；保留原始深度量纲。
        fmt = PixelFormat(plane.pixel_format).name
        array = bytes(plane.data) if fmt == 'MJPG' else plane.numpy().copy(order='C')
        meta = {
            'width': plane.width, 'height': plane.height,
            'pixel_format_str': {'Z16': 'DEPTH16'}.get(fmt, fmt),
            'depth_scale_m': plane.depth_scale_m,
            'capture_id': plane.capture_id,
            'capture_monotonic_ns': frame.meta.capture_monotonic_ns,
            'publish_epoch_ns': plane.publish_epoch_ns,
            'source_id': self.output.source_id,
            'descriptor': self.descriptor,
        }
        self.last_key = key
        self.sequence += 1
        self.last_frame = frame
        return (array, self.sequence, meta) if with_meta else (array, self.sequence)

    def reset_frame_position(self):
        self.last_key = None

    def get_intrinsics(self):
        p, = [p for p in self.descriptor['planes'] if p['sensor_id'] == self.output.sensor_id]
        intr = dict(p['intrinsics'])
        intr['coeffs'] = intr.pop('coefficients')
        return intr

    def close(self):
        if self.reader is not None:
            self.reader.close()
            self.reader = None


def open_camera_shm_consumer(output, *, name=None):
    return CameraConsumer(output)


def get_camera_shm_output(module_name, output_name):
    outputs = list_camera_shm_outputs('autolife_s1', 'robot_v3_1', module_name=module_name)
    output, = [o for o in outputs if o.output_name.lower() == output_name.lower()]
    reader = CameraConsumer(output)
    try:
        intr = reader.get_intrinsics()
        return replace(output, width=int(intr['width']), height=int(intr['height']))
    finally:
        reader.close()


def open_rgbd_pair(robot_model='autolife_s1', robot_version='robot_v3_1',
                   module_name='mod_camera_rgbd_head', primary='color'):
    """提供旧抓取读取接口，但彩色与深度只从同一次原子发布取得。"""
    outputs = list_camera_shm_outputs(robot_model, robot_version, module_name=module_name)
    color, = [o for o in outputs if o.output_name == 'color']
    depth, = [o for o in outputs if o.output_name == 'depth']
    if (color.channel_name, color.source_id) != (depth.channel_name, depth.source_id):
        raise ValueError('RGB-D需要来自同一个已对齐共享内存通道')
    if primary not in ('color', 'depth'):
        raise ValueError('primary must be color or depth')
    first, second = (color, depth) if primary == 'color' else (depth, color)
    consumer = CameraConsumer(first)
    if consumer.descriptor.get('alignment') != 'depth_to_color':
        consumer.close()
        raise ValueError('RGB-D通道尚未对齐深度到彩色图像')
    shared = {'secondary': None}

    class PlaneView:
        def __init__(self, output):
            self.output = output

        def get_intrinsics(self):
            p, = [p for p in consumer.descriptor['planes'] if p['sensor_id'] == self.output.sensor_id]
            intr = dict(p['intrinsics'])
            intr['coeffs'] = intr.pop('coefficients')
            return intr

        def get_latest(self, *, nonblock=False, timeout=.5, with_meta=False):
            if self.output == second:
                pair = shared['secondary']
                shared['secondary'] = None
                return pair if pair is None or with_meta else pair[:2]
            shared['secondary'] = None
            result = consumer.get_latest(nonblock=nonblock, timeout=timeout, with_meta=True)
            if result is None:
                return None
            array, ident, meta = result
            frame = consumer.last_frame
            plane, = [p for p in frame.planes if p.sensor_id == second.sensor_id]
            depth_plane, = [p for p in frame.planes if p.sensor_id == depth.sensor_id]
            if plane.capture_id != meta['capture_id']:
                raise ValueError('RGB-D发布内的采集编号不一致')
            if depth_plane.depth_scale_m is None or abs(depth_plane.depth_scale_m - .001) > 1e-8:
                raise ValueError('当前抓取需要毫米单位深度，请显式配置深度换算')
            other_meta = dict(meta, width=plane.width, height=plane.height,
                              pixel_format_str='DEPTH16' if second == depth else 'BGR',
                              depth_scale_m=plane.depth_scale_m)
            shared['secondary'] = (plane.numpy().copy(order='C'), ident, other_meta)
            return (array, ident, meta) if with_meta else (array, ident)

        def close(self):
            consumer.close()

    return [PlaneView(color), PlaneView(depth)]
