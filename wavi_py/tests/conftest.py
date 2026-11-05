"""Suite-wide: point the sync settings (QSettings) and the sync state file at a
temp dir, so no test (the MainWindow ones included: they make a
SyncController) reads or writes the user's own cluster settings and queue."""

import os
import tempfile


def pytest_configure(config):
    d = tempfile.mkdtemp(prefix="wavi_sync_suite_")
    os.environ.setdefault("WAVI_SETTINGS_INI", os.path.join(d, "settings.ini"))
    os.environ.setdefault("WAVI_SYNC_STATE_DIR", os.path.join(d, "state"))
