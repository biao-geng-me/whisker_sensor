"""The Analysis timeline's activity row: the recording card's preview as an
optional Timeline variable (``rec:card``), timeline only, on top, re-binned to the
span shown, and the same numbers as the card (``replay/card.py``).

Run: QT_QPA_PLATFORM=offscreen python -m pytest wavi_py/tests/test_timeline_card.py
"""

from __future__ import annotations

import numpy as np

from wavi_py.daq.analysis import CARD_KEY, card_image
from wavi_py.daq.series import WSK
from wavi_py.replay.card import quantise, rms_db
from wavi_py.tests.test_analysis_r2 import _pump, _window, app, no_modal_errors, win  # noqa: F401
from wavi_py.views.analysis_timeline import timeline_source_of
from wavi_py.views.analysis_window import PLOTS, TIMELINE


def test_card_row(app, win):
    aw, _sels = _window(app, win)
    aw.show()
    tl = aw.timeline
    assert any(v.key == CARD_KEY and not v.plots for v in aw._vars)       # timeline only
    aw.set_checked([CARD_KEY], True, PLOTS)
    assert CARD_KEY not in aw.checked_keys()
    aw.set_checked([CARD_KEY], True, TIMELINE)
    assert _pump(app, lambda: tl._card is not None, 3)
    assert list(tl.rows)[0] == "card"                                      # on top
    q, a, b = tl._card
    assert q.dtype == np.uint8 and q.shape[0] == 32 and q.shape[1] >= 16
    assert tl._card_img.isVisible()

    # the card's numbers over the same span
    s = timeline_source_of(aw._source_fn)
    t, y = s.series.get(WSK, slice(None), a, b, aw.model.despike)
    ref = quantise(rms_db(t - a, y, b - a, q.shape[1]))
    assert np.array_equal(ref, q)

    # zoomed in: finer bins over the shorter span
    tl._anchor.setXRange(a + 0.25 * (b - a), a + 0.5 * (b - a), padding=0)
    assert _pump(app, lambda: tl._card[2] - tl._card[1] < 0.5 * (b - a), 3)
    g, db = tl.card_at(0.5 * (tl._card[1] + tl._card[2]), 0.5)
    assert g == 0 and -40.0 <= db <= 15.0

    aw.set_checked([CARD_KEY], False, TIMELINE)
    assert _pump(app, lambda: tl._card is None and "card" not in tl.rows, 3)
    assert card_image(s, b + 10, b + 20, 32) is None                      # no data there
    aw.close()
