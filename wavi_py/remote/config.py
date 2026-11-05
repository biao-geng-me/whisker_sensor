"""Sync settings and where the sync state lives.

The settings are in QSettings (PyQt6's QtCore, no QApplication needed), so the
GUI's "Cluster..." dialog and ``tools/wavi_sync.py`` read the same values:
the registry on Windows, a plist on macOS, an ini file on Linux. Keys sit under
``cluster/``. ``WAVI_SETTINGS_INI`` points them at an ini file instead (tests).

The queue and the per-file uploaded state are a small JSON file in the user's
data directory (``state_dir()``), not in QSettings: it changes every upload and
both processes rewrite it whole. ``WAVI_SYNC_STATE_DIR`` overrides it.
"""

from __future__ import annotations

import os
import sys
from dataclasses import asdict, dataclass, fields
from pathlib import Path

DEFAULT_ROOT = "/shared/rc/whiskers/rov-recordings"
DEFAULT_KEEP = 10
APP = "wavi_py"


@dataclass
class SyncConfig:
    host: str = ""                    # the user's ~/.ssh/config alias, or user@host
    root: str = DEFAULT_ROOT          # the remote root; YYYY-MM-DD/ folders below it
    auto_upload: bool = False         # upload each recording after Stop
    prune: bool = False               # delete local copies beyond the newest keep_n
    keep_n: int = DEFAULT_KEEP
    local_dir: str = ""               # the GUI's output dir (the CLI's default)
    prune_confirmed: bool = False     # the GUI asked once, and the user said yes

    @property
    def configured(self) -> bool:
        return bool(self.host.strip() and self.root.strip())


def _settings():
    from PyQt6.QtCore import QSettings
    ini = os.environ.get("WAVI_SETTINGS_INI")
    if ini:
        return QSettings(ini, QSettings.Format.IniFormat)
    return QSettings(APP, APP)


def _as(kind, v, default):
    if v is None:
        return default
    if kind is bool:
        return v if isinstance(v, bool) else str(v).lower() in ("1", "true", "yes")
    if kind is int:
        try:
            return int(v)
        except (TypeError, ValueError):
            return default
    return str(v)


def load() -> SyncConfig:
    s, d = _settings(), SyncConfig()
    kw = {}
    for f in fields(SyncConfig):
        default = getattr(d, f.name)
        kw[f.name] = _as(type(default), s.value(f"cluster/{f.name}"), default)
    cfg = SyncConfig(**kw)
    cfg.keep_n = max(1, cfg.keep_n)
    return cfg


def save(cfg: SyncConfig) -> None:
    s = _settings()
    for k, v in asdict(cfg).items():
        s.setValue(f"cluster/{k}", v)
    s.sync()


def state_dir() -> Path:
    """Per-user data dir: %LOCALAPPDATA%\\wavi_py, ~/Library/Application
    Support/wavi_py, or $XDG_DATA_HOME (~/.local/share)/wavi_py."""
    env = os.environ.get("WAVI_SYNC_STATE_DIR")
    if env:
        p = Path(env)
    elif sys.platform == "win32":
        p = Path(os.environ.get("LOCALAPPDATA") or Path.home() / "AppData" / "Local") / APP
    elif sys.platform == "darwin":
        p = Path.home() / "Library" / "Application Support" / APP
    else:
        p = Path(os.environ.get("XDG_DATA_HOME") or Path.home() / ".local" / "share") / APP
    p.mkdir(parents=True, exist_ok=True)
    return p


def default_local_dir() -> Path:
    """The GUI's default output dir (main_window._default_outpath), uncreated."""
    home = os.environ.get("USERPROFILE") or os.environ.get("HOME") or "."
    return Path(home) / "wavi_data"
