"""Notes in the GUI (offscreen): Mark bad and Add note on a replayed file and on a
local recording, the timeline's note markers (hover, click to go there) and
bad cue, the notes list (edit = del + new, delete), and a note made while
recording landing at the same time on the recording's replay timeline.
Screenshots go to $WAVI_TEST_SHOTS if set.

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_notes_gui.py
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from PyQt6.QtWidgets import QApplication, QInputDialog, QMessageBox  # noqa: E402

from wavi_py.replay import notes  # noqa: E402
from wavi_py.tests import replay_fixture as fx  # noqa: E402

SHOTS = os.environ.get("WAVI_TEST_SHOTS")


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture(autouse=True)
def no_modal_errors(monkeypatch):
    def critical(parent, title, text, *a, **k):
        raise AssertionError(f"{title}: {text}")
    monkeypatch.setattr(QMessageBox, "critical", critical)


def _pump(app, pred, secs: float) -> bool:
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        app.processEvents()
        if pred():
            return True
        time.sleep(0.01)
    app.processEvents()
    return pred()


def _shot(widget, name: str) -> None:
    if SHOTS:
        Path(SHOTS).mkdir(parents=True, exist_ok=True)
        widget.grab().save(str(Path(SHOTS) / f"{name}.png"))


@pytest.fixture
def replay_win(app, tmp_path):
    from wavi_py.main_window import MainWindow
    p = tmp_path / "st_2026-11-02_1644_57.91_pool.jsonl"
    fx.write_recording(p)
    w = MainWindow(transport="replay", nsensor=16, replay_path=str(p), outpath=tmp_path / "out")
    w.resize(900, 560)
    w.show()
    w.open_replay(str(p))
    assert _pump(app, lambda: w._client is not None and w._replay.active, 10)
    w._path = p
    yield w
    w.close()


def _answer_note(monkeypatch, text, pinned=True):
    import wavi_py.main_window as mwmod
    asked = []

    def fake(parent, title, t, text0="", pinned0=True):
        asked.append(t)
        return text, (t if pinned else None)
    monkeypatch.setattr(mwmod, "ask_note", fake)
    return asked


def test_replay_notes_and_flag(app, replay_win, monkeypatch):
    w, p = replay_win, replay_win._path
    npath = p.with_name("st_2026-11-02_1644_57.91_pool.notes.jsonl")
    assert w._notes.recording == p and w._note_btn.isEnabled() and w._flag_btn.isEnabled()
    assert w._flag_btn.text() == "Mark bad" and npath.name in w._flag_btn.toolTip()
    bar = w._timeline_docks[0].widget()
    assert bar.notes_btn.text() == "Notes (0)"
    # Add note at the playhead (N), and a whole-recording one
    w._replay.seek(4.0)
    assert _pump(app, lambda: abs(w._replay.position - 4.0) < 0.05, 5)
    asked = _answer_note(monkeypatch, "touch W3")
    w.add_note()
    assert asked == [pytest.approx(4.0, abs=0.05)]
    _answer_note(monkeypatch, "pool test, calm", pinned=False)
    w.add_note()
    st = notes.load(npath)
    assert [(n["text"], n["t"] is None) for n in st.notes] == [("touch W3", False),
                                                              ("pool test, calm", True)]
    assert st.notes[0]["by"] == w._who
    # the strip: one marker at 4 s; the button counts both
    assert _pump(app, lambda: len(bar.strip._note_x) == 1, 2)
    assert bar.strip._note_x[0] == pytest.approx(4.0, abs=0.05)
    assert bar.notes_btn.text() == "Notes (2)"
    # hover shows the text; a click next to the marker goes to the note's time
    shown = []
    import wavi_py.views.replay_bar as rb
    monkeypatch.setattr(rb.QToolTip, "showText", lambda pos, text, *a: shown.append(text))
    vb = bar.strip.getPlotItem().getViewBox()
    px = vb.viewPixelSize()[0]
    from PyQt6.QtCore import QPointF
    sp = vb.mapViewToScene(QPointF(4.0 + 2 * px, 0.0))
    bar.strip._hover(sp)
    assert shown and "touch W3" in shown[-1]

    class Click:
        def button(self):
            from PyQt6.QtCore import Qt
            return Qt.MouseButton.LeftButton

        def double(self):
            return False

        def scenePos(self):
            return sp
    w._replay.seek(8.0)
    assert _pump(app, lambda: abs(w._replay.position - 8.0) < 0.05, 5)
    bar.strip._clicked(Click())
    assert _pump(app, lambda: abs(w._replay.position - st.notes[0]["t"]) < 0.02, 5)
    # Mark bad: a flag line with the reason; the label, the tooltip and the strip follow
    monkeypatch.setattr(QInputDialog, "getText", lambda *a, **k: ("W7 loose", True))
    w._flag_btn.click()
    assert notes.load(npath).bad and notes.load(npath).reason == "W7 loose"
    assert w._flag_btn.text() == "Unmark bad" and "W7 loose" in w._flag_btn.toolTip()
    assert bar.strip.bad_label.isVisibleTo(bar) and "BAD" in bar.notes_btn.text()
    _shot(w, "notes_replay_bad")
    # the notes list: edit (del + new) and delete
    dlg = w.show_notes()
    assert dlg.tree.topLevelItemCount() == 2 and "marked bad" in dlg.head.text()
    import wavi_py.views.notes_panel as npmod
    monkeypatch.setattr(npmod, "ask_note", lambda *a, **k: ("touch W3, hard", 4.5))
    dlg.tree.setCurrentItem(dlg.tree.topLevelItem(0))
    dlg._edit()
    st = notes.load(npath)
    assert [n["text"] for n in st.notes] == ["touch W3, hard", "pool test, calm"]
    assert st.notes[0]["t"] == 4.5
    _shot(dlg, "notes_list")
    monkeypatch.setattr(QMessageBox, "question", lambda *a, **k: QMessageBox.StandardButton.Yes)
    dlg.tree.setCurrentItem(dlg.tree.topLevelItem(1))
    dlg._delete()
    assert [n["text"] for n in notes.load(npath).notes] == ["touch W3, hard"]
    evs = [ln["ev"] for ln in notes.read(npath)]
    assert evs == ["note", "note", "flag", "del", "note", "del"]          # nothing rewritten
    # double-click a pinned note: go there
    dlg._go(dlg.tree.topLevelItem(0))
    assert _pump(app, lambda: abs(w._replay.position - 4.5) < 0.02, 5)
    # another window appends: picked up by the poll
    notes.append(npath, notes.note("from the other window", t=1.0, by="other"))
    assert _pump(app, lambda: len(bar.strip._note_x) == 2, 4)
    # Unmark: keep true
    w._flag_btn.click()
    assert not notes.load(npath).bad and w._flag_btn.text() == "Mark bad"
    assert not bar.strip.bad_label.isVisibleTo(bar)
    # closing the replay: nothing to note on
    w._disconnect()
    assert w._notes.recording is None and not w._note_btn.isEnabled()


def test_local_recording_notes_on_replay_time(app, tmp_path, monkeypatch):
    """Sim (a local .jsonl tee): a note while recording, pinned to the copy's time;
    Mark bad after the stop goes to that recording's notes; its replay puts the
    note where the stream was when it was made."""
    from wavi_py.main_window import MainWindow
    from wavi_py.replay.index import RecordingIndex
    w = MainWindow(transport="sim", nsensor=16, outpath=tmp_path)
    w.show()
    try:
        w._connect()
        assert _pump(app, lambda: w._record_btn.isEnabled(), 5)
        assert not w._note_btn.isEnabled()
        w._tag_edit.setText("live-notes")
        w._record_btn.click()
        assert _pump(app, lambda: w._is_recording, 3)
        assert w._note_btn.isEnabled()
        time.sleep(1.5)
        app.processEvents()
        asked = _answer_note(monkeypatch, "now")
        r = w._client._reader
        w.add_note()
        t_rx = r._last_t                                  # the stream's clock at the note
        (t_note,) = asked
        assert 1.0 < t_note < 3.0
        time.sleep(0.5)
        w._record_btn.click()
        assert _pump(app, lambda: not w._is_recording, 3)
        (rec,) = sorted(tmp_path.glob("st_*_live-notes.jsonl"))
        assert w._notes.recording.name.startswith(rec.name[:-6])
        monkeypatch.setattr(QInputDialog, "getText", lambda *a, **k: ("sim", True))
        w._flag_btn.click()                               # the last recording, after the stop
        st = notes.load(notes.notes_path(rec))
        assert st.bad and [n["text"] for n in st.notes] == ["now"]
        # the same time on the recording's replay clock (session zero = first line)
        idx = RecordingIndex.build(rec)
        t_replay = t_rx - idx.t0
        print(f"note t {t_note:.3f} s, replay time of that line {t_replay:.3f} s")
        assert abs(st.notes[0]["t"] - t_replay) < 0.1
    finally:
        w._disconnect()
        w.close()
