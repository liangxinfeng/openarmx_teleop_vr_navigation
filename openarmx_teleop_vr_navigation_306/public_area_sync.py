"""Upload 283 public-area missions to the hotel admin API.

The navigation bridge owns one ``HotelMissionService``.  This module watches its
snapshot, builds the JSON body documented in
``docs/public-area-tasks-api.md`` (``POST /api/admin/hotels/:hotelId/public-area-tasks``)
and posts it from a private thread.

Design invariants
-----------------
* Nothing here can start, stop, pause or steer a mission.  The operator starts
  the task from the VR menu; this module only reacts to a committed record.
* No request is made before the mission record is committed (``record_committed``
  and an existing mission directory).  A task chain that never starts uploads
  nothing.
* One worker thread, one request in flight.  A failure is logged, kept for retry
  and never raised into the ROS executor or the mission.
* Re-posting the same ``task_id`` overwrites the record server-side, so partial
  progress updates and retries are idempotent by contract.
* The admin token is read from a 0600 file and never logged, stored in the
  record, or echoed in errors.
"""

from __future__ import annotations

import base64
import hashlib
import ipaddress
import json
import logging
import os
import re
import threading
import time
import traceback
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Mapping

API_PATH = "/api/admin/hotels/{hotel_id}/public-area-tasks"
HOTEL_ID_PATTERN = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
DEFAULT_UPLOAD_CHAINS = ("酒店公区巡查",)

MAX_REQUEST_BYTES = 64 * 1024 * 1024
MAX_PHOTO_BYTES = 8 * 1024 * 1024
MAX_TOTAL_PHOTO_BYTES = 32 * 1024 * 1024
MAX_PHOTOS = 200
MAX_REPORT_BYTES = 1024 * 1024
MAX_RESPONSE_BYTES = 256 * 1024
DISABLED_CHECK_SECONDS = 5.0
WORKER_TICK_SECONDS = 0.25
SAMPLE_SECONDS = 1.0
RETRY_MIN_SECONDS = 2.0
RETRY_MAX_SECONDS = 60.0

STAGE_LABELS = {
    "cleaning": "清理作业",
    "basket_pickup": "取垃圾篮",
    "disposal": "丢弃垃圾",
    "dumping": "倾倒垃圾",
    "navigation": "导航",
    "return_to_dock": "返回充电桩",
    "charging": "充电确认",
    "NAVIGATING_TO_AREA": "前往点位",
    "NAVIGATING_TO_BIN": "前往垃圾桶",
    "NAVIGATING_TO_PICKUP": "前往前台",
    "RETURNING_TO_DOCK": "返回充电桩",
    "PICKING_BASKET": "取垃圾篮",
    "DUMPING": "倾倒垃圾",
}
RESULT_LABELS = {
    "completed": "已完成",
    "no_target": "无目标物",
    "skipped": "已跳过",
    "failed": "未完成",
    "running": "进行中",
}
ITEM_LABELS = {
    "water_bottle": "水瓶",
    "water_cup": "水杯",
    "tissue": "纸巾",
    "paper": "纸张",
    "other": "其他",
}
PHOTO_LABELS = {
    "before": "清理前照片",
    "after": "清理后照片",
    "supplemental": "补充照片",
}
MIME_BY_SUFFIX = {
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".png": "image/png",
    ".webp": "image/webp",
    ".bmp": "image/bmp",
}


class ConfigError(ValueError):
    """The sync configuration cannot be used as written."""


class _LogAdapter:
    """Make rclpy's RcutilsLogger accept ``logger.info("x=%s", value)``.

    RcutilsLogger only accepts a single message argument and has no
    ``exception()``; formatting arguments or calling ``exception()`` raises
    TypeError and would kill the upload thread.
    """

    def __init__(self, logger: Any) -> None:
        self._logger = logger

    @staticmethod
    def _message(message: Any, args: tuple[Any, ...]) -> str:
        text = str(message)
        if not args:
            return text
        try:
            return text % args
        except (TypeError, ValueError):
            return text + " " + " ".join(str(item) for item in args)

    def info(self, message: Any, *args: Any) -> None:
        self._logger.info(self._message(message, args))

    def warning(self, message: Any, *args: Any) -> None:
        self._logger.warning(self._message(message, args))

    def error(self, message: Any, *args: Any) -> None:
        self._logger.error(self._message(message, args))

    def debug(self, message: Any, *args: Any) -> None:
        self._logger.debug(self._message(message, args))

    def exception(self, message: Any, *args: Any) -> None:
        self._logger.error(self._message(message, args) + "\n" + traceback.format_exc())


class UploadError(RuntimeError):
    """A single upload attempt failed."""

    def __init__(self, message: str, *, status: int | None = None,
                 code: str = "", permanent: bool = False) -> None:
        super().__init__(message)
        self.status = status
        self.code = code
        self.permanent = permanent


# ---------------------------------------------------------------------------
# configuration
# ---------------------------------------------------------------------------


def validate_base_url(value: str, *, allow_lan: bool = False) -> str:
    """Reject anything that is not a plain http(s) base URL for a real host."""

    text = str(value or "").strip()
    if not text:
        raise ConfigError("base_url 为空")
    parsed = urllib.parse.urlsplit(text)
    if parsed.scheme not in {"http", "https"}:
        raise ConfigError("base_url 只支持 http/https")
    if not parsed.hostname:
        raise ConfigError("base_url 缺少主机名")
    if parsed.username or parsed.password:
        raise ConfigError("base_url 不允许携带用户名密码")
    if parsed.query or parsed.fragment:
        raise ConfigError("base_url 不允许带 query 或 fragment")
    host = parsed.hostname.lower()
    if host in {"localhost", "localhost.localdomain"} and not allow_lan:
        raise ConfigError("base_url 不允许指向本机（allow_lan_addresses=true 可放行）")
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        address = None
    if address is not None and not allow_lan and (
        address.is_loopback or address.is_private or address.is_link_local
        or address.is_reserved or address.is_multicast
    ):
        raise ConfigError("base_url 不允许指向环回/内网地址（allow_lan_addresses=true 可放行）")
    if parsed.scheme == "http" and not allow_lan:
        raise ConfigError("base_url 生产环境必须使用 https")
    return text.rstrip("/")


@dataclass(frozen=True)
class SyncConfig:
    enabled: bool = False
    base_url: str = ""
    hotel_id: str = ""
    token_file: str = ""
    admin_token: str = ""
    robot_id: str = "283"
    upload_chains: tuple[str, ...] = DEFAULT_UPLOAD_CHAINS
    request_timeout_seconds: float = 10.0
    min_upload_interval_seconds: float = 2.0
    heartbeat_seconds: float = 30.0
    upload_photos: bool = True
    allow_lan_addresses: bool = False
    source_tag: str = "vr283-bridge"
    record_root: str = "~/Documents/sushi_task_records"
    reconcile_seconds: float = 60.0
    stale_running_seconds: float = 1800.0
    reconcile_since: str = ""
    track_enabled: bool = True
    track_sample_seconds: float = 2.0
    track_max_points: int = 6000
    track_write_seconds: float = 5.0

    @property
    def api_url(self) -> str:
        return self.base_url + API_PATH.format(hotel_id=self.hotel_id)


def load_config(path: str | Path) -> SyncConfig:
    source = Path(path).expanduser()
    payload = json.loads(source.read_text(encoding="utf-8-sig"))
    if not isinstance(payload, Mapping):
        raise ConfigError("配置文件必须是 JSON 对象")

    def flag(name: str, default: bool) -> bool:
        value = payload.get(name, default)
        return bool(value)

    def number(name: str, default: float, low: float, high: float) -> float:
        try:
            value = float(payload.get(name, default))
        except (TypeError, ValueError):
            raise ConfigError(f"{name} 必须是数字") from None
        return max(low, min(high, value))

    chains = payload.get("upload_chains") or list(DEFAULT_UPLOAD_CHAINS)
    if isinstance(chains, str):
        chains = [chains]
    chains = tuple(str(x).strip() for x in chains if str(x).strip())

    enabled = flag("enabled", False)
    base_url = str(payload.get("base_url") or "").strip()
    hotel_id = str(payload.get("hotel_id") or "").strip()
    allow_lan = flag("allow_lan_addresses", False)

    if enabled:
        if not HOTEL_ID_PATTERN.match(hotel_id):
            raise ConfigError("hotel_id 只能包含字母、数字、下划线和短横线（1-64 位）")
        base_url = validate_base_url(base_url, allow_lan=allow_lan)
        if not payload.get("token_file") and not payload.get("admin_token"):
            raise ConfigError("必须提供 token_file 或 admin_token")
    elif base_url:
        try:
            base_url = validate_base_url(base_url, allow_lan=allow_lan)
        except ConfigError:
            base_url = ""

    return SyncConfig(
        enabled=enabled,
        base_url=base_url,
        hotel_id=hotel_id,
        token_file=str(payload.get("token_file") or "").strip(),
        admin_token=str(payload.get("admin_token") or "").strip(),
        robot_id=str(payload.get("robot_id") or "283").strip() or "283",
        upload_chains=chains or DEFAULT_UPLOAD_CHAINS,
        request_timeout_seconds=number("request_timeout_seconds", 10.0, 1.0, 60.0),
        min_upload_interval_seconds=number("min_upload_interval_seconds", 2.0, 0.0, 60.0),
        heartbeat_seconds=number("heartbeat_seconds", 30.0, 5.0, 600.0),
        upload_photos=flag("upload_photos", True),
        allow_lan_addresses=allow_lan,
        source_tag=str(payload.get("source_tag") or "vr283-bridge").strip() or "vr283-bridge",
        record_root=str(payload.get("record_root") or "~/Documents/sushi_task_records").strip(),
        reconcile_seconds=number("reconcile_seconds", 60.0, 5.0, 3600.0),
        stale_running_seconds=number("stale_running_seconds", 1800.0, 60.0, 86400.0),
        reconcile_since=str(payload.get("reconcile_since") or "").strip(),
        track_enabled=flag("track_enabled", True),
        track_sample_seconds=number("track_sample_seconds", 2.0, 0.5, 60.0),
        track_max_points=int(number("track_max_points", 6000, 200, 50000)),
        track_write_seconds=number("track_write_seconds", 5.0, 1.0, 300.0),
    )


class TokenProvider:
    """Read the bearer token from a 0600 file, re-reading it when it changes."""

    def __init__(self, config: SyncConfig) -> None:
        self._config = config
        self._cached = ""
        self._stamp: tuple[float, int] | None = None

    def get(self) -> str:
        inline = str(self._config.admin_token or "").strip()
        if inline:
            return inline
        path = Path(self._config.token_file).expanduser()
        try:
            stat = path.stat()
        except OSError as error:
            raise UploadError(f"读取 token 文件失败：{error.strerror or error}", permanent=True)
        stamp = (stat.st_mtime, stat.st_size)
        if stamp == self._stamp and self._cached:
            return self._cached
        try:
            token = path.read_text(encoding="utf-8").strip()
        except OSError as error:
            raise UploadError(f"读取 token 文件失败：{error.strerror or error}", permanent=True)
        if len(token) < 16:
            raise UploadError("token 文件内容无效（少于 16 字符）", permanent=True)
        self._stamp = stamp
        self._cached = token
        return token


# ---------------------------------------------------------------------------
# payload mapping
# ---------------------------------------------------------------------------


def mission_outcome(summary: Mapping[str, Any]) -> str:
    if not summary.get("ended_at"):
        return "running"
    status = str(summary.get("execution_status") or summary.get("status") or "").strip()
    result = str(summary.get("result") or "").strip()
    if status == "aborted" or result == "aborted":
        return "aborted"
    if result == "failed":
        return "failed"
    if result == "completed":
        return "completed"
    return "unknown"


def _text(value: Any) -> str:
    return "" if value is None else str(value).strip()


def _now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def _synthesize_aborted(summary: Mapping[str, Any]) -> dict[str, Any]:
    """Close a mission that never wrote ``ended_at`` (crash, power loss)."""

    result = json.loads(json.dumps(summary, ensure_ascii=False, default=str))
    result["ended_at"] = _now_iso()
    result["status"] = "aborted"
    result["execution_status"] = "aborted"
    result["result"] = "aborted"
    result["data_quality"] = "incomplete"
    result["review_required"] = True
    failures = result.setdefault("failures", [])
    failures.append({
        "code": "ROBOT_INTERRUPTED",
        "stage": "mission",
        "detail": "机器人进程中断，任务未正常结束",
        "recoverable": False,
    })
    return result


def _stage_text(stage: str) -> str:
    return STAGE_LABELS.get(stage, stage or "未知环节")


def _cleaned_items_text(items: Any) -> str:
    if not isinstance(items, (list, tuple)):
        return ""
    parts: list[str] = []
    for item in items:
        if not isinstance(item, Mapping):
            continue
        label = _text(item.get("label"))
        if not label:
            continue
        try:
            count = int(item.get("count", 1))
        except (TypeError, ValueError):
            count = 1
        parts.append(f"{ITEM_LABELS.get(label, label)} × {max(1, count)}")
    return "、".join(parts)


def _iter_area_steps(summary: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    """Accept both the bridge compat view (``areas``) and a raw mission.json."""

    areas = summary.get("areas")
    if isinstance(areas, list) and areas:
        return [item for item in areas if isinstance(item, Mapping)]
    return [
        item for item in (summary.get("steps") or [])
        if isinstance(item, Mapping) and _text(item.get("kind")) == "cleaning"
    ]


def _iter_images(summary: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    artifacts = summary.get("artifacts") or {}
    images = artifacts.get("images") if isinstance(artifacts, Mapping) else None
    if isinstance(images, list) and images:
        return [item for item in images if isinstance(item, Mapping)]
    collected: list[Mapping[str, Any]] = []
    for step in _iter_area_steps(summary):
        for photo in step.get("photos") or []:
            if isinstance(photo, Mapping) and _text(photo.get("status")) == "captured":
                entry = dict(photo)
                entry.setdefault("area_id", _text(step.get("area_id")))
                collected.append(entry)
    return collected


def _area_entries(summary: Mapping[str, Any]) -> tuple[list[dict[str, Any]], dict[str, str]]:
    entries: list[dict[str, Any]] = []
    names: dict[str, str] = {}
    for step in _iter_area_steps(summary):
        area_id = _text(step.get("area_id"))
        name = _text(step.get("display_name")) or _text(step.get("name")) or area_id
        if not name:
            continue
        if area_id:
            names[area_id] = name
        result = _text(step.get("result"))
        if not result:
            result = "completed" if _text(step.get("status")) == "finished" else "running"
        entry: dict[str, Any] = {"area_id": area_id or name, "name": name, "result": result}
        label = RESULT_LABELS.get(result)
        if label:
            entry["result_label"] = label
        cleaned = _cleaned_items_text(step.get("cleaned_items"))
        if cleaned:
            entry["cleaned_items"] = cleaned
        note = _text(step.get("note"))
        if note:
            entry["note"] = note
        if result in {"skipped", "failed"}:
            entry["failure"] = f"清理作业（{note or result}）"
        entries.append(entry)
    return entries, names


def _failure_entries(summary: Mapping[str, Any], area_names: Mapping[str, str]) -> tuple[list[str], list[dict[str, str]]]:
    lines: list[str] = []
    details: list[dict[str, str]] = []
    for item in summary.get("failures") or []:
        if not isinstance(item, Mapping):
            continue
        code = _text(item.get("code")) or "UNKNOWN"
        stage = _text(item.get("stage")) or _text(item.get("phase")) or "unknown"
        area = area_names.get(_text(item.get("area_id")), "")
        detail = _text(item.get("detail")) or _text(item.get("result"))
        label = _stage_text(stage)
        lines.append(f"{area}·{label}：{detail}" if area else f"{label}：{detail}")
        entry = {"code": code, "stage": stage, "detail": detail}
        if area:
            entry["area"] = area
        details.append(entry)
    return lines, details


def _encode_photo(data: bytes, suffix: str, *, shrink: bool = True) -> bytes | None:
    if len(data) <= MAX_PHOTO_BYTES:
        return data
    if not shrink:
        return None
    try:  # Pillow is optional; a missing dependency must not break the upload.
        from io import BytesIO

        from PIL import Image
    except Exception:  # pragma: no cover - depends on the robot environment
        return None
    try:
        with Image.open(BytesIO(data)) as image:
            image = image.convert("RGB")
            image.thumbnail((1920, 1920))
            buffer = BytesIO()
            image.save(buffer, format="JPEG", quality=82, optimize=True)
            resized = buffer.getvalue()
    except Exception:
        return None
    return resized if len(resized) <= MAX_PHOTO_BYTES else None


def _photo_entries(
    summary: Mapping[str, Any],
    area_names: Mapping[str, str],
    reader: Callable[[str], bytes | None] | None,
    *,
    upload: bool,
) -> tuple[list[dict[str, str]], int, int]:
    """Return (photos, bytes_used, dropped)."""

    if not upload or reader is None:
        return [], 0, 0
    ordered = sorted(
        _iter_images(summary),
        key=lambda item: (_text(item.get("stage")) == "supplemental", _text(item.get("captured_at"))),
    )
    photos: list[dict[str, str]] = []
    used = 0
    dropped = 0
    for item in ordered:
        if len(photos) >= MAX_PHOTOS:
            dropped += 1
            continue
        relative = _text(item.get("path"))
        if not relative:
            continue
        area_name = area_names.get(_text(item.get("area_id")), "")
        if not area_name:
            dropped += 1
            continue
        try:
            data = reader(relative)
        except OSError:
            data = None
        if not data:
            dropped += 1
            continue
        encoded = _encode_photo(data, Path(relative).suffix.lower())
        if encoded is None or used + len(encoded) > MAX_TOTAL_PHOTO_BYTES:
            dropped += 1
            continue
        stage = _text(item.get("stage")) or "photo"
        photos.append({
            "label": PHOTO_LABELS.get(stage, "现场照片"),
            "stage": stage,
            "area_name": area_name,
            "file_name": Path(relative).name,
            "data_url": "data:%s;base64,%s" % (
                MIME_BY_SUFFIX.get(Path(relative).suffix.lower(), "image/jpeg"),
                base64.b64encode(encoded).decode("ascii"),
            ),
        })
        used += len(encoded)
    return photos, used, dropped


def build_task_payload(
    summary: Mapping[str, Any],
    *,
    mission: Mapping[str, Any] | None = None,
    report_markdown: str | None = None,
    photo_reader: Callable[[str], bytes | None] | None = None,
    upload_photos: bool = True,
    source_tag: str = "vr283-bridge",
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Map a MissionRecorder summary to the public-area-tasks JSON object."""

    task_id = _text(summary.get("task_id"))
    if not task_id:
        raise ValueError("mission summary has no task_id")
    areas, area_names = _area_entries(summary)
    lines, details = _failure_entries(summary, area_names)
    photos, photo_bytes, dropped = _photo_entries(
        summary, area_names, photo_reader, upload=upload_photos
    )

    payload: dict[str, Any] = {
        "task_id": task_id,
        "robot_id": _text(summary.get("robot_id")) or "283",
        "started_at": _text(summary.get("started_at")),
        "outcome": mission_outcome(summary),
        "mission_status": _text(summary.get("execution_status")) or _text(summary.get("status")),
        "mission_result": _text(summary.get("result")),
        "data_quality": _text(summary.get("data_quality")) or "complete",
        "review_required": bool(summary.get("review_required")),
        "failures": lines,
        "failure_details": details,
        "areas": areas,
        "photos": photos,
        "source_file": source_tag,
    }
    ended_at = _text(summary.get("ended_at"))
    if ended_at:
        payload["ended_at"] = ended_at
    for key in ("start_waypoint", "return_waypoint"):
        value = _text(summary.get(key))
        if value:
            payload[key] = value
    if report_markdown:
        payload["report_markdown"] = report_markdown[:MAX_REPORT_BYTES]
    if mission is not None:
        payload["mission"] = dict(mission)
    return payload, {"photo_bytes": photo_bytes, "dropped_photos": dropped}


def payload_signature(summary: Mapping[str, Any]) -> str:
    parts: list[str] = [
        _text(summary.get("task_id")),
        _text(summary.get("current_state")),
        _text(summary.get("ended_at")),
        _text(summary.get("execution_status")),
        _text(summary.get("result")),
        _text(summary.get("data_quality")),
        str(bool(summary.get("review_required"))),
    ]
    for step in summary.get("areas") or []:
        if not isinstance(step, Mapping):
            continue
        parts.append("|".join(
            _text(step.get(key)) for key in ("area_id", "status", "result", "failure_stage", "note")
        ) + f"#{len(step.get('photos') or [])}")
    parts.append(f"failures={len(summary.get('failures') or [])}")
    parts.append(f"images={len((summary.get('artifacts') or {}).get('images') or [])}")
    return hashlib.sha256("\n".join(parts).encode("utf-8")).hexdigest()[:32]


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):  # noqa: D401
        return None


def _read_limited(stream, limit: int) -> bytes:
    chunks: list[bytes] = []
    remaining = limit
    while remaining > 0:
        chunk = stream.read(min(remaining, 65536))
        if not chunk:
            break
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _error_from_body(body: bytes) -> tuple[str, str]:
    try:
        payload = json.loads(body.decode("utf-8", "replace"))
    except (ValueError, UnicodeDecodeError):
        return "", ""
    if not isinstance(payload, Mapping):
        return "", ""
    return _text(payload.get("code")), _text(payload.get("message"))


class _HttpClient:
    def __init__(self, config: SyncConfig) -> None:
        handlers: list[Any] = [urllib.request.ProxyHandler({}), _NoRedirect()]
        self._opener = urllib.request.build_opener(*handlers)
        self._timeout = config.request_timeout_seconds

    def post(self, url: str, body: bytes, token: str) -> dict[str, Any]:
        if len(body) > MAX_REQUEST_BYTES:
            raise UploadError("请求体超过 64MB", permanent=False)
        request = urllib.request.Request(
            url,
            data=body,
            method="POST",
            headers={
                "Authorization": f"Bearer {token}",
                "Content-Type": "application/json; charset=utf-8",
                "Accept": "application/json",
                "User-Agent": "vr283-public-area-sync/1.0",
            },
        )
        try:
            with self._opener.open(request, timeout=self._timeout) as response:
                status = int(response.status)
                raw = _read_limited(response, MAX_RESPONSE_BYTES)
        except urllib.error.HTTPError as error:
            raw = _read_limited(error, MAX_RESPONSE_BYTES)
            code, message = _error_from_body(raw)
            status = int(error.code)
            permanent = status in {301, 302, 303, 307, 308, 400, 401, 403, 404}
            raise UploadError(
                message or f"HTTP {status}",
                status=status,
                code=code,
                permanent=permanent,
            ) from None
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            raise UploadError(f"网络错误：{getattr(error, 'reason', error)}") from None
        if status not in {200, 201}:
            code, message = _error_from_body(raw)
            raise UploadError(message or f"HTTP {status}", status=status, code=code)
        try:
            payload = json.loads(raw.decode("utf-8", "replace")) if raw else {}
        except ValueError:
            payload = {}
        return payload if isinstance(payload, dict) else {}


# ---------------------------------------------------------------------------
# worker
# ---------------------------------------------------------------------------


@dataclass
class _Job:
    task_id: str
    signature: str
    summary: dict[str, Any]
    mission_dir: str
    final: bool
    mission_override: dict[str, Any] | None = None
    created_at: float = field(default_factory=time.monotonic)
    next_attempt: float = 0.0
    attempts: int = 0
    drop_photos: bool = False
    prebuilt: dict[str, Any] | None = None
    from_outbox: bool = False


class PublicAreaSync:
    """Watch one hotel mission snapshot and upload it to the admin API."""

    def __init__(
        self,
        config_path: str | Path,
        *,
        logger: logging.Logger | None = None,
        clock: Callable[[], float] = time.monotonic,
        track_factory: Callable[..., Any] | None = None,
    ) -> None:
        self._config_path = Path(config_path).expanduser()
        self._log = _LogAdapter(logger or logging.getLogger("public_area_sync"))
        self._clock = clock
        self._lock = threading.RLock()
        self._config: SyncConfig | None = None
        self._config_stamp: tuple[float, int] | None = None
        self._config_error = ""
        self._tokens: TokenProvider | None = None
        self._client: _HttpClient | None = None
        self._jobs: dict[str, _Job] = {}
        self._last_ok: dict[str, str] = {}
        self._last_upload_at: dict[str, float] = {}
        # Permanent rejections (400/401/403/404) are not retried until the
        # snapshot signature or the configuration changes.
        self._blocked: dict[str, str] = {}
        self._state: dict[str, Any] = {
            "enabled": False,
            "hotel_id": "",
            "task_id": "",
            "state": "disabled",
            "last_http": None,
            "uploaded_at": None,
            "attempts": 0,
            "last_error": "",
            "pending": False,
        }
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self._provider: Callable[[], Mapping[str, Any]] | None = None
        self._samples = 0
        # Durable terminal-state outbox: a mission that reached the server as
        # "running" must end terminal even if the process dies mid-mission.
        self._record_root: Path | None = None
        self._epoch = ""
        self._active_task = ""
        self._last_reconcile = 0.0
        self._track_factory = track_factory
        self._track = None
        self._track_provider: Callable[[], Mapping[str, Any]] | None = None
        self._last_track_write = 0.0

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        config = self._current_config()  # surface config errors in /api/navigation/state
        if config is not None:
            self._prepare_outbox(config)
            self._load_outbox()
            try:
                # A mission cannot survive a bridge restart: any record without
                # ended_at at startup is orphaned and must be closed now.
                self._reconcile(config, assume_abandoned=True)
            except Exception as error:  # noqa: BLE001
                self._log.warning("public-area startup reconcile failed: %s", error)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="public-area-sync", daemon=True)
        self._thread.start()
        self._log.info("public-area sync worker started")

    def stop(self, timeout: float = 5.0) -> None:
        self._stop.set()
        self._wake.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None
        self._log.info("public-area sync worker stopped")

    # -- called from the ROS timer ----------------------------------------
    def set_snapshot_provider(
        self, provider: Callable[[], Mapping[str, Any]] | None
    ) -> None:
        """Sample the hotel service from the sync thread, not the ROS executor.

        ``HotelMissionService.snapshot()`` takes the mission lock (it can wait
        for a photo capture), so it must never run on the executor thread that
        also drives the 50 ms teleop tick.
        """
        self._provider = provider

    def set_track_provider(self, provider: Callable[[], Mapping[str, Any]] | None) -> None:
        """Map/pose/waypoint context for the track recorder (sync thread only)."""
        self._track_provider = provider

    def observe(self, snapshot: Mapping[str, Any]) -> None:
        """Queue an upload for a hotel snapshot. Never blocks, never raises."""

        try:
            config = self._current_config()
            if config is None or not config.enabled:
                self._set_state(state="disabled", enabled=False, pending=False)
                return
            mission = snapshot.get("mission") or {}
            record_dir = _text(snapshot.get("record_directory"))
            if not mission or not record_dir:
                return
            committed = snapshot.get("record_committed")
            if committed is None:
                # Defensive: an older bridge snapshot may omit the flag even
                # though the record is already on disk.
                committed = (Path(record_dir) / "mission.json").is_file()
            if not committed:
                return
            if not self._chain_allowed(mission, config):
                return
            task_id = _text(mission.get("task_id"))
            if not task_id:
                return
            signature = payload_signature(mission)
            final = bool(_text(mission.get("ended_at")))
            if self._record_root is None:
                try:
                    self._record_root = Path(record_dir).parents[1]
                except (IndexError, OSError):
                    self._record_root = None
            self._active_task = "" if final else task_id
            with self._lock:
                if self._blocked.get(task_id) == signature:
                    return
                if self._last_ok.get(task_id) == signature:
                    last = self._last_upload_at.get(task_id, 0.0)
                    heartbeat = config.heartbeat_seconds
                    if final or not heartbeat or (self._clock() - last) < heartbeat:
                        return
                existing = self._jobs.get(task_id)
                if existing is not None and existing.signature == signature:
                    if final and not existing.final:
                        existing.final = True
                        self._wake.set()
                    return
                self._jobs[task_id] = _Job(
                    task_id=task_id,
                    signature=signature,
                    summary=json.loads(json.dumps(mission, ensure_ascii=False, default=str)),
                    mission_dir=record_dir,
                    final=final,
                )
                self._set_state(task_id=task_id, state="queued", pending=True)
            self._log.info(
                "public-area sync queued task=%s state=%s final=%s",
                task_id, _text(mission.get("current_state")), final,
            )
            self._wake.set()
        except Exception as error:  # pragma: no cover - defensive
            self._log.warning("public-area sync observe failed: %s", error)

    # -- status ------------------------------------------------------------
    def status(self) -> dict[str, Any]:
        with self._lock:
            payload = dict(self._state)
            payload["samples"] = self._samples
        config = self._config
        if config is not None:
            payload["hotel_id"] = config.hotel_id
            payload["enabled"] = bool(config.enabled)
        if self._config_error:
            payload["last_error"] = self._config_error[:300]
        return payload

    # -- durable outbox / reconciliation ----------------------------------
    def _outbox_dir(self) -> Path:
        root = self._record_root or Path("~/Documents/sushi_task_records").expanduser()
        return root / ".public_area_sync"

    def _prepare_outbox(self, config: SyncConfig) -> None:
        if self._record_root is None:
            self._record_root = Path(config.record_root).expanduser()
        try:
            self._outbox_dir().mkdir(parents=True, exist_ok=True)
        except OSError as error:
            self._log.warning("cannot create public-area outbox: %s", error)
        self._epoch = config.reconcile_since or self._load_epoch()

    def _load_epoch(self) -> str:
        path = self._outbox_dir() / "state.json"
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            epoch = _text(payload.get("epoch"))
            if epoch:
                return epoch
        except (OSError, ValueError):
            pass
        epoch = _now_iso()
        try:
            self._outbox_dir().mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps({"epoch": epoch}, ensure_ascii=False), encoding="utf-8")
        except OSError:
            pass
        return epoch

    def _job_path(self, task_id: str) -> Path:
        return self._outbox_dir() / f"{task_id}.json"

    def _ack_path(self, task_id: str) -> Path:
        return self._outbox_dir() / f"{task_id}.ack"

    def _load_outbox(self) -> None:
        directory = self._outbox_dir()
        if not directory.is_dir():
            return
        for path in sorted(directory.glob("*.json")):
            if path.name == "state.json":
                continue
            try:
                stored = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            payload = stored.get("payload")
            task_id = _text(stored.get("task_id")) or path.stem
            if not isinstance(payload, dict) or not task_id:
                continue
            with self._lock:
                self._jobs[task_id] = _Job(
                    task_id=task_id,
                    signature=_text(stored.get("signature")) or "outbox",
                    summary={},
                    mission_dir=_text(stored.get("mission_dir")),
                    final=True,
                    prebuilt=payload,
                    from_outbox=True,
                )
                self._set_state(task_id=task_id, state="queued", pending=True)
            self._log.info("public-area sync resumed outbox task=%s", task_id)

    def _store_outbox(self, job: _Job, payload: Mapping[str, Any]) -> None:
        path = self._job_path(job.task_id)
        try:
            directory = self._outbox_dir()
            directory.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps({
                "task_id": job.task_id,
                "signature": job.signature,
                "mission_dir": job.mission_dir,
                "payload": payload,
                "saved_at": _now_iso(),
            }, ensure_ascii=False), encoding="utf-8")
            os.replace(temporary, path)
        except OSError as error:
            self._log.warning("cannot store public-area outbox: %s", error)

    def _ack_outbox(self, job: _Job) -> None:
        try:
            self._ack_path(job.task_id).write_text(
                json.dumps({"task_id": job.task_id, "acked_at": _now_iso()}, ensure_ascii=False),
                encoding="utf-8",
            )
        except OSError as error:
            self._log.warning("cannot write public-area ack: %s", error)
        try:
            self._job_path(job.task_id).unlink()
        except FileNotFoundError:
            pass
        except OSError as error:
            self._log.warning("cannot clear public-area outbox: %s", error)

    def _reconcile(self, config: SyncConfig, *, assume_abandoned: bool = False) -> None:
        """Finish missions whose terminal snapshot was never delivered."""
        root = self._record_root
        if root is None or not root.is_dir():
            return
        now = time.time()
        day_dirs = sorted(
            (item for item in root.iterdir() if item.is_dir() and item.name.isdigit()),
            reverse=True,
        )[:3]
        for day in day_dirs:
            for mission_dir in sorted(day.iterdir()):
                summary_path = mission_dir / "mission.json"
                if not summary_path.is_file():
                    continue
                try:
                    summary = json.loads(summary_path.read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    continue
                task_id = _text(summary.get("task_id"))
                if not task_id or task_id == self._active_task:
                    continue
                with self._lock:
                    if task_id in self._jobs or task_id in self._last_ok:
                        continue
                if self._ack_path(task_id).exists():
                    continue
                if not self._chain_allowed(summary, config):
                    continue
                started = _text(summary.get("started_at"))
                if self._epoch and started and started < self._epoch:
                    continue
                override = None
                if not _text(summary.get("ended_at")):
                    if not assume_abandoned:
                        try:
                            age = now - summary_path.stat().st_mtime
                        except OSError:
                            continue
                        if age < config.stale_running_seconds:
                            continue
                    summary = _synthesize_aborted(summary)
                    override = summary
                    self._log.warning(
                        "public-area reconcile: mission %s looks abandoned, closing as aborted", task_id
                    )
                with self._lock:
                    self._jobs[task_id] = _Job(
                        task_id=task_id,
                        signature="reconcile:" + task_id,
                        summary=json.loads(json.dumps(summary, ensure_ascii=False, default=str)),
                        mission_dir=str(mission_dir),
                        final=True,
                        mission_override=override,
                    )
                    self._set_state(task_id=task_id, state="queued", pending=True)
                self._log.info("public-area reconcile queued task=%s", task_id)

    # -- internals ---------------------------------------------------------
    def _current_config(self) -> SyncConfig | None:
        try:
            stat = self._config_path.stat()
        except OSError as error:
            self._config_error = f"配置文件不可读：{error.strerror or error}"
            return None
        stamp = (stat.st_mtime, stat.st_size)
        with self._lock:
            if self._config is not None and stamp == self._config_stamp:
                return self._config
        try:
            config = load_config(self._config_path)
        except (ConfigError, ValueError, OSError) as error:
            self._config_error = f"配置无效：{error}"
            self._log.warning("public-area sync config rejected: %s", error)
            with self._lock:
                self._config = None
                self._config_stamp = stamp
            return None
        with self._lock:
            changed = self._config != config
            self._config = config
            self._config_stamp = stamp
            self._config_error = ""
            if changed:
                self._tokens = TokenProvider(config)
                self._client = _HttpClient(config)
                self._blocked.clear()
                self._track = (
                    self._track_factory(
                        sample_seconds=config.track_sample_seconds,
                        max_points=config.track_max_points,
                    )
                    if (self._track_factory is not None and config.track_enabled)
                    else None
                )
        return config

    def _chain_allowed(self, mission: Mapping[str, Any], config: SyncConfig) -> bool:
        metadata = mission.get("metadata") or {}
        mission_config = metadata.get("config") or {}
        chain = _text(mission_config.get("source_chain"))
        if chain:
            # The editable TXT chain is the authoritative link between a
            # template and the task the operator actually started.
            return chain in set(config.upload_chains)
        names = {
            _text(mission_config.get("name")),
            _text(mission.get("mission_config_id")),
        }
        return bool(names & set(config.upload_chains))

    def _set_state(self, **fields: Any) -> None:
        with self._lock:
            self._state.update(fields)

    def _sample_track(self, snapshot: Mapping[str, Any]) -> None:
        track = self._track
        provider = self._track_provider
        if track is None or provider is None:
            return
        try:
            context = provider()
        except Exception as error:  # noqa: BLE001 - never break the mission
            self._log.warning("public-area track context failed: %s", error)
            return
        track.update(snapshot, context)
        if not track.dirty:
            return
        config = self._config
        interval = config.track_write_seconds if config is not None else 5.0
        mission = snapshot.get("mission") or {}
        final = bool(_text(mission.get("ended_at")))
        now = self._clock()
        if not final and (now - self._last_track_write) < interval:
            return
        directory = _text(snapshot.get("record_directory"))
        if not directory:
            return
        payload = track.payload()
        if not payload:
            return
        try:
            target = Path(directory) / "track.json"
            temporary = target.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
            os.replace(temporary, target)
        except OSError as error:
            self._log.warning("cannot write track.json: %s", error)
            return
        track.clear_dirty()
        self._last_track_write = now

    def _run(self) -> None:
        last_sample = 0.0
        while not self._stop.is_set():
            try:
                config = self._current_config()
                if config is not None and config.enabled:
                    now = self._clock()
                    if now - self._last_reconcile >= config.reconcile_seconds:
                        self._last_reconcile = now
                        try:
                            self._reconcile(config)
                        except Exception as error:  # noqa: BLE001
                            self._log.warning("public-area reconcile failed: %s", error)
                provider = self._provider
                now = self._clock()
                if provider is not None and now - last_sample >= SAMPLE_SECONDS:
                    last_sample = now
                    try:
                        snapshot = provider()
                    except Exception as error:  # noqa: BLE001
                        self._log.warning("public-area snapshot read failed: %s", error)
                    else:
                        self._samples += 1
                        if snapshot:
                            self.observe(snapshot)
                            self._sample_track(snapshot)
                job = self._take_due_job()
                if job is not None:
                    self._process(job)
            except Exception as error:  # pragma: no cover - defensive
                self._log.exception("public-area sync worker error: %s", error)
            self._wake.wait(WORKER_TICK_SECONDS)
            self._wake.clear()

    def _take_due_job(self) -> _Job | None:
        now = self._clock()
        with self._lock:
            due = [job for job in self._jobs.values() if job.next_attempt <= now]
            if not due:
                return None
            job = min(due, key=lambda item: (item.next_attempt, item.created_at))
            self._jobs.pop(job.task_id, None)
            return job

    def _process(self, job: _Job) -> None:
        config = self._current_config()
        if config is None or not config.enabled:
            return
        if config.min_upload_interval_seconds and not job.final:
            last = self._last_upload_at.get(job.task_id, 0.0)
            wait = config.min_upload_interval_seconds - (self._clock() - last)
            if wait > 0:
                job.next_attempt = self._clock() + wait
                with self._lock:
                    self._jobs.setdefault(job.task_id, job)
                return

        self._set_state(task_id=job.task_id, state="uploading", pending=True)
        try:
            payload = job.prebuilt or self._build(job, config)
            if job.final and not job.from_outbox:
                # Write-ahead: the terminal body survives a crash between here
                # and the server acknowledgement.
                self._store_outbox(job, payload)
            body = json.dumps({"tasks": [payload]}, ensure_ascii=False).encode("utf-8")
            if len(body) > MAX_REQUEST_BYTES:
                raise UploadError("请求体超过 64MB", permanent=False)
            token = self._tokens.get() if self._tokens else ""
            client = self._client or _HttpClient(config)
            response = client.post(config.api_url, body, token)
            self._on_success(job, config, response, len(body))
        except UploadError as error:
            if error.status == 413 and not job.drop_photos:
                job.drop_photos = True
                job.next_attempt = self._clock()
                job.attempts += 1
                with self._lock:
                    self._jobs[job.task_id] = job
                self._log.warning("upload too large, retrying without photos: %s", job.task_id)
                return
            self._on_failure(job, config, error)
        except Exception as error:  # pragma: no cover - defensive
            self._on_failure(job, config, UploadError(f"未预期错误：{error}"))

    def _build(self, job: _Job, config: SyncConfig) -> dict[str, Any]:
        mission_dir = Path(job.mission_dir)
        summary = job.summary
        report = ""
        try:
            report_path = mission_dir / "report.md"
            if report_path.is_file():
                report = report_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            report = ""
        mission_payload: dict[str, Any] | None = None
        try:
            mission_path = mission_dir / "mission.json"
            if mission_path.is_file():
                loaded = json.loads(mission_path.read_text(encoding="utf-8"))
                if isinstance(loaded, dict):
                    mission_payload = loaded
        except (OSError, ValueError):
            mission_payload = None
        if job.mission_override:
            mission_payload = job.mission_override
        try:
            track_path = mission_dir / "track.json"
            if track_path.is_file():
                track = json.loads(track_path.read_text(encoding="utf-8"))
                if isinstance(track, dict):
                    mission_payload = dict(mission_payload or {})
                    mission_payload["track"] = track
        except (OSError, ValueError):
            pass

        def reader(relative: str) -> bytes | None:
            try:
                return (mission_dir / relative).read_bytes()
            except OSError:
                return None

        payload, meta = build_task_payload(
            summary,
            mission=mission_payload,
            report_markdown=report,
            photo_reader=reader,
            upload_photos=config.upload_photos and not job.drop_photos,
            source_tag=config.source_tag,
        )
        if job.drop_photos or meta["dropped_photos"]:
            payload["data_quality"] = "incomplete"
            self._log.warning(
                "public-area upload drops photos task=%s drop_all=%s dropped=%s",
                job.task_id, job.drop_photos, meta["dropped_photos"],
            )
        return payload

    def _on_success(self, job: _Job, config: SyncConfig, response: Mapping[str, Any], size: int) -> None:
        now = self._clock()
        with self._lock:
            self._last_ok[job.task_id] = job.signature
            self._last_upload_at[job.task_id] = now
            self._blocked.pop(job.task_id, None)
            self._jobs.pop(job.task_id, None)
            self._set_state(
                task_id=job.task_id,
                state="ok",
                last_http=201,
                uploaded_at=datetime.now().astimezone().isoformat(timespec="seconds"),
                attempts=0,
                last_error="",
                pending=False,
            )
        self._clear_pending(job, config)
        if job.final:
            self._ack_outbox(job)
        skipped = response.get("skipped") if isinstance(response, Mapping) else None
        self._log.info(
            "public-area upload ok task=%s bytes=%s final=%s skipped=%s",
            job.task_id, size, job.final, skipped if skipped is not None else "[]",
        )

    def _on_failure(self, job: _Job, config: SyncConfig, error: UploadError) -> None:
        job.attempts += 1
        delay = min(RETRY_MAX_SECONDS, RETRY_MIN_SECONDS * (2 ** min(job.attempts - 1, 5)))
        job.next_attempt = self._clock() + delay
        message = str(error)
        with self._lock:
            self._set_state(
                task_id=job.task_id,
                state="blocked" if error.permanent else "retry",
                last_http=error.status,
                attempts=job.attempts,
                last_error=message[:300],
                pending=True,
            )
            if not error.permanent:
                self._jobs[job.task_id] = job
            else:
                self._blocked[job.task_id] = job.signature
        self._log.warning(
            "public-area upload failed task=%s status=%s code=%s permanent=%s: %s",
            job.task_id, error.status, error.code, error.permanent, message,
        )
        if error.permanent:
            self._write_pending(job, config, message)

    # -- persistence -------------------------------------------------------
    def _state_dir(self, mission_dir: Path) -> Path | None:
        try:
            return mission_dir.parent.parent / ".public_area_sync"
        except (IndexError, OSError):
            return None

    def _write_pending(self, job: _Job, config: SyncConfig, message: str) -> None:
        directory = self._state_dir(Path(job.mission_dir))
        if directory is None:
            return
        try:
            directory.mkdir(parents=True, exist_ok=True)
            target = directory / f"{job.task_id}.json"
            temporary = target.with_suffix(".json.tmp")
            temporary.write_text(json.dumps({
                "task_id": job.task_id,
                "signature": job.signature,
                "hotel_id": config.hotel_id,
                "attempts": job.attempts,
                "last_error": message[:500],
                "final": job.final,
                "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            os.replace(temporary, target)
        except OSError as error:
            self._log.warning("cannot persist pending upload state: %s", error)

    def _clear_pending(self, job: _Job, config: SyncConfig) -> None:
        directory = self._state_dir(Path(job.mission_dir))
        if directory is None:
            return
        target = directory / f"{job.task_id}.json"
        try:
            if target.exists():
                target.unlink()
        except OSError:
            pass
