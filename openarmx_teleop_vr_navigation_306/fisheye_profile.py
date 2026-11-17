"""Validation and persistence helpers for the v3 fisheye profile."""

from __future__ import annotations

import copy
import json
import math
import os
import tempfile
from collections.abc import Mapping
from pathlib import Path
from typing import Any


DEFAULT_PROFILE_PATH = Path.home() / ".config" / "autolife" / "fisheye360" / "vr_profile.json"

EDITABLE_KEYS = (
    "cx",
    "cy",
    "radius",
    "focal",
    "pitch",
    "yaw",
    "roll",
    "height",
    "lowerCut",
)

_CAMERAS = ("front", "rear")
_MAX_PROFILE_BYTES = 64 * 1024

# These are the browser ranges in v3/app.js. Yaw is checked as an offset from
# the camera's default value because the stored profile keeps absolute yaw.
_UI_RANGES = {
    "cx": (260, 375),
    "cy": (135, 220),
    "radius": (130, 187),
    "focal": (75, 130),
    "pitch": (-35, 45),
    "yaw": (-40, 40),
    "roll": (-20, 20),
    "height": (0.8, 1.8),
    "lowerCut": (80, 185),
}


def _is_finite_number(value: Any) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError):
        return False


def _validate_defaults(defaults: Mapping[str, Any]) -> None:
    if not isinstance(defaults, Mapping):
        raise ValueError("defaults must be a mapping")

    revision = defaults.get("revision")
    if not isinstance(revision, str) or not revision:
        raise ValueError("defaults must contain a non-empty revision")

    for camera in _CAMERAS:
        config = defaults.get(camera)
        if not isinstance(config, Mapping):
            raise ValueError(f"defaults must contain a {camera} mapping")
        for key in EDITABLE_KEYS:
            if key not in config or not _is_finite_number(config[key]):
                raise ValueError(f"defaults.{camera}.{key} must be finite numeric data")


def _copy_defaults(defaults: Mapping[str, Any]) -> dict[str, Any]:
    _validate_defaults(defaults)
    try:
        return copy.deepcopy(dict(defaults))
    except (TypeError, ValueError) as error:
        raise ValueError("defaults cannot be copied") from error


def _validate_payload_keys(payload: Mapping[str, Any], defaults: Mapping[str, Any]) -> None:
    allowed_top_level = set(defaults) | {"revision"}
    extra_top_level = set(payload) - allowed_top_level
    if extra_top_level:
        raise ValueError("profile contains unknown top-level data")

    for camera in _CAMERAS:
        config = payload.get(camera)
        if not isinstance(config, Mapping):
            raise ValueError(f"profile must contain a {camera} mapping")

        allowed_camera_keys = set(defaults[camera])
        extra_camera_keys = set(config) - allowed_camera_keys
        if extra_camera_keys:
            raise ValueError(f"profile.{camera} contains unknown data")

        missing = [key for key in EDITABLE_KEYS if key not in config]
        if missing:
            raise ValueError(f"profile.{camera} is missing editable data")


def validate_profile(payload: Mapping[str, Any], defaults: Mapping[str, Any]) -> dict[str, Any]:
    """Validate a profile and return defaults with editable values overlaid.

    The accepted payload may contain just ``revision``, ``front`` and ``rear``
    or may be a full profile captured from the v3 schema. Known immutable
    fields are accepted and ignored; unknown fields are rejected.
    """

    normalized = _copy_defaults(defaults)
    if not isinstance(payload, Mapping):
        raise ValueError("profile must be a mapping")

    _validate_payload_keys(payload, defaults)
    if payload.get("revision") != defaults["revision"]:
        raise ValueError("profile revision does not match defaults")

    for camera in _CAMERAS:
        source = payload[camera]
        target = normalized[camera]
        for key in EDITABLE_KEYS:
            value = source[key]
            if not _is_finite_number(value):
                raise ValueError(f"profile.{camera}.{key} must be finite numeric data")

            minimum, maximum = _UI_RANGES[key]
            comparable = value - defaults[camera]["yaw"] if key == "yaw" else value
            if comparable < minimum or comparable > maximum:
                raise ValueError(f"profile.{camera}.{key} is outside the v3 UI range")
            target[key] = value

    return normalized


def _encode_profile(profile: Mapping[str, Any]) -> bytes:
    try:
        text = json.dumps(
            profile,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
    except (TypeError, ValueError) as error:
        raise ValueError("normalized profile is not JSON-compatible") from error

    data = (text + "\n").encode("utf-8")
    if len(data) > _MAX_PROFILE_BYTES:
        raise ValueError("profile is too large")
    return data


def save_profile(
    payload: Mapping[str, Any],
    defaults: Mapping[str, Any],
    path: os.PathLike[str] | str = DEFAULT_PROFILE_PATH,
) -> dict[str, Any]:
    """Validate and atomically save a normalized profile."""

    normalized = validate_profile(payload, defaults)
    data = _encode_profile(normalized)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)

    descriptor: int | None = None
    temporary: Path | None = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{target.name}.",
            suffix=".tmp",
            dir=target.parent,
        )
        temporary = Path(temporary_name)
        try:
            os.chmod(temporary, 0o600)
        except OSError:
            pass

        with os.fdopen(descriptor, "wb") as handle:
            descriptor = None
            handle.write(data)
            handle.flush()
            try:
                os.fsync(handle.fileno())
            except OSError:
                pass

        os.replace(temporary, target)
        temporary = None
        try:
            os.chmod(target, 0o600)
        except OSError:
            pass
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass

    return normalized


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON value {value!r} is not allowed")


def load_profile(
    defaults: Mapping[str, Any],
    path: os.PathLike[str] | str = DEFAULT_PROFILE_PATH,
) -> tuple[dict[str, Any], str]:
    """Load a valid saved profile, or return defaults with source ``default``."""

    fallback = _copy_defaults(defaults)
    target = Path(path)
    try:
        if not target.is_file() or target.stat().st_size > _MAX_PROFILE_BYTES:
            return fallback, "default"
        with target.open("rb") as handle:
            raw = handle.read(_MAX_PROFILE_BYTES + 1)
        if len(raw) > _MAX_PROFILE_BYTES:
            return fallback, "default"

        payload = json.loads(
            raw.decode("utf-8"),
            parse_constant=_reject_json_constant,
        )
        return validate_profile(payload, defaults), "synced"
    except (
        OSError,
        UnicodeError,
        ValueError,
        TypeError,
        RecursionError,
    ):
        return fallback, "default"


__all__ = [
    "DEFAULT_PROFILE_PATH",
    "EDITABLE_KEYS",
    "load_profile",
    "save_profile",
    "validate_profile",
]
