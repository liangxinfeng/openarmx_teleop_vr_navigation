import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading

import pytest

from openarmx_teleop_vr_navigation_306.recognition_provider import configured_recognizer


def test_http_provider_sends_photo_metadata_and_validates_result(tmp_path):
    received = []
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            received.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
            data = json.dumps({'objects':[{'label':'water_bottle','count':2,'confidence':0.9}],
                               'view_quality':'good'}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        def log_message(self, *args): pass
    server = ThreadingHTTPServer(('127.0.0.1',0), Handler)
    thread = threading.Thread(target=server.serve_forever,daemon=True)
    thread.start()
    try:
        path = tmp_path / 'test.jpg'
        path.write_bytes(b'jpeg')
        recognize = configured_recognizer({'provider':'http_json',
            'endpoint':f'http://127.0.0.1:{server.server_port}/analyze'})
        result = recognize(path, {'stage':'before'})
        assert result['objects'][0]['count'] == 2
        assert received[0]['image_base64'] == 'anBlZw=='
        assert received[0]['metadata']['stage'] == 'before'
    finally:
        server.shutdown()
        thread.join(2)
        server.server_close()


@pytest.mark.parametrize('config', [{'provider':'unknown'}, {'provider':'http_json','endpoint':'file:///image'},
    {'provider':'http_json','endpoint':'http://localhost','timeout_seconds':0}])
def test_invalid_provider_config_fails_before_mission(config):
    with pytest.raises(ValueError): configured_recognizer(config)
