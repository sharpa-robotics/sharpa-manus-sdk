"""Bootstrap bundled SharpaWave SDK (python + native lib) for retargeting scripts."""

from __future__ import annotations

import os
import sys
from typing import List, Optional

SDK_VERSION = "5.0.4"
SDK_DIRNAME = f"SharpaWaveSDK_{SDK_VERSION}"


def bundled_sdk_root() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), SDK_DIRNAME)


def bundled_python_path() -> str:
    return os.path.join(bundled_sdk_root(), "python")


def bundled_lib_path() -> str:
    return os.path.join(bundled_sdk_root(), "lib")


def default_sharpa_python_paths() -> List[str]:
    """Search order: env override, bundled copy, system install."""
    return [
        os.environ.get("SHARPA_WAVE_SDK_PYTHON", ""),
        bundled_python_path(),
        "/opt/sharpa-wave-sdk/python",
    ]


def setup_sharpa_sdk(extra_python_paths: Optional[List[str]] = None) -> str:
    """
    Prepend SDK python path and extend LD_LIBRARY_PATH for libsharpa-wave-sdk.so.
    Returns the python path that was prioritized (first existing candidate).
    """
    candidates = list(extra_python_paths or []) + default_sharpa_python_paths()
    valid_paths: List[str] = []
    for path in candidates:
        if not path or not os.path.isdir(path):
            continue
        if path not in valid_paths:
            valid_paths.append(path)

    # Insert highest-priority path last so it ends up at sys.path[0].
    for path in reversed(valid_paths):
        if path not in sys.path:
            sys.path.insert(0, path)

    lib_dirs = []
    for lib_dir in (bundled_lib_path(), "/opt/sharpa-wave-sdk/lib"):
        if os.path.isdir(lib_dir) and lib_dir not in lib_dirs:
            lib_dirs.append(lib_dir)

    if lib_dirs:
        current = os.environ.get("LD_LIBRARY_PATH", "")
        parts = [p for p in current.split(os.pathsep) if p]
        for lib_dir in reversed(lib_dirs):
            if lib_dir not in parts:
                parts.insert(0, lib_dir)
        os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(parts)

    return valid_paths[0] if valid_paths else ""
