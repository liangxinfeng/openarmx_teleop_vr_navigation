"""User-local, domain-isolated recorder RPC. This module has no motor interfaces."""
import http.client
import json
import os
from pathlib import Path
import socket


def namespace(robot_id):
    robot_id = str(robot_id)
    if not robot_id.isdigit():
        raise ValueError('invalid robot id')
    domain = str(int(os.environ.get('ROS_DOMAIN_ID', '0')))
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}'))
    return runtime, f'robot-teaching-{domain}-{robot_id}'


def discovery_path(robot_id):
    root, name = namespace(robot_id)
    return root / (name + '.json')


class UnixConnection(http.client.HTTPConnection):
    def __init__(self, path, timeout=1.):
        super().__init__('localhost', timeout=timeout)
        self.path = str(path)

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


def request(robot_id, route='/status', payload=None, timeout=1., desktop=False):
    root, prefix = namespace(robot_id)
    info = json.loads(discovery_path(robot_id).read_text())
    path = Path(info['socket'])
    if path.parent.resolve() != root.resolve() or not path.name.startswith(prefix + '-'):
        raise ValueError('invalid recorder socket')
    conn = UnixConnection(path, timeout)
    try:
        data = None if payload is None else json.dumps(payload).encode()
        conn.request('GET' if data is None else 'POST', route, data,
                     {'Content-Type': 'application/json', 'X-Recorder-Token': info['token'],
                      'X-Recorder-Client':'desktop' if desktop else 'vr'})
        response = conn.getresponse()
        raw = response.read(2 * 1024 * 1024 + 1)
        if len(raw) > 2 * 1024 * 1024:
            raise ValueError('recorder response too large')
        result = json.loads(raw)
        if response.status != 200 or not result.get('success', True):
            raise ValueError(result.get('message', '录制服务没有完成请求'))
        return result
    finally:
        conn.close()
