"""Optional HTTP image-analysis adapter; no credentials or models bundled."""
import base64
import json
import os
import urllib.request
from urllib.parse import urlparse


LABELS = {"water_bottle", "water_cup", "tissue", "paper", "other"}


def validate_recognition_result(payload):
    if not isinstance(payload, dict):
        raise ValueError('recognition response must be an object')
    result = dict(payload)
    if result.get('status', 'completed') not in {'completed', 'not_configured', 'pending_review'}:
        raise ValueError('invalid recognition status')
    objects = result.get('objects', [])
    if not isinstance(objects, list) or len(objects) > 1000:
        raise ValueError('objects must be a bounded list')
    normalized = []
    for entry in objects:
        if not isinstance(entry, dict) or entry.get('label') not in LABELS:
            raise ValueError('invalid object label')
        count = entry.get('count', 1)
        confidence = entry.get('confidence')
        if isinstance(count, bool) or not isinstance(count, int) or not 0 <= count <= 1000:
            raise ValueError('object count must be an integer from 0 to 1000')
        if confidence is not None and (isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or not 0 <= confidence <= 1):
            raise ValueError('confidence must be between 0 and 1')
        normalized.append({**entry, 'count': count})
    result['objects'] = normalized
    result.setdefault('status', 'completed')
    result.setdefault('cleaning_result', 'pending_review')
    result.setdefault('view_quality', 'unknown')
    return result


def configured_recognizer(config):
    provider = config.get('provider', 'disabled')
    if provider == 'disabled':
        return None
    if provider != 'http_json':
        raise ValueError('recognition provider must be disabled or http_json')
    endpoint = str(config.get('endpoint', '')).strip()
    if urlparse(endpoint).scheme not in {'http', 'https'} or not urlparse(endpoint).netloc:
        raise ValueError('recognition endpoint must be an explicit HTTP(S) URL')
    timeout = float(config.get('timeout_seconds', 20))
    if not 0 < timeout <= 120:
        raise ValueError('recognition timeout must be between 0 and 120 seconds')
    token_env = str(config.get('token_env', '')).strip()

    def recognize(image_path, metadata):
        data = image_path.read_bytes()
        if len(data) > 20 * 1024 * 1024:
            raise ValueError('recognition image exceeds 20 MiB')
        headers = {'Content-Type': 'application/json'}
        if token_env:
            token = os.environ.get(token_env)
            if not token:
                raise RuntimeError('recognition token environment variable is not set')
            headers['Authorization'] = 'Bearer ' + token
        body = json.dumps({'schema_version': '1.0', 'image_base64': base64.b64encode(data).decode('ascii'),
                           'mime_type': 'image/jpeg', 'metadata': dict(metadata),
                           'labels': sorted(LABELS)}, ensure_ascii=False).encode('utf-8')
        request = urllib.request.Request(endpoint, data=body, headers=headers, method='POST')
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read(1024 * 1024 + 1)
        if len(raw) > 1024 * 1024:
            raise ValueError('recognition response exceeds 1 MiB')
        return validate_recognition_result(json.loads(raw))

    return recognize
