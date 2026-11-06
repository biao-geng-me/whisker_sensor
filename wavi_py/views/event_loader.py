"""The Events mode's background work: the query, the stores, the overlay.

``EventLoader.request(query, variables)`` runs, in one worker thread at a time:
``EventSet.from_query`` (the sidecars; ``setReady``), then the recordings'
stores (``StoreCache``: ``progress("loading 3 of 8 recordings ...")``, a cold
build ~2.5 s per 5 min recording), the event selections and the
``eventset.overlay`` (traces, aggregates; ``overlayReady``). Only the newest
request counts: a request made while one runs waits for it and replaces any
other waiting one; a result of an older request is dropped. The GUI thread only
draws what arrives, so a cold store build never freezes it.

A request with the same query (variables changed) reuses the event set and the
selections; a query change reuses the stores (the cache), so only the sidecars
are read again.
"""

from __future__ import annotations

import threading
import traceback
from typing import Optional

from PyQt6.QtCore import QObject, pyqtSignal

from wavi_py.replay import eventset as evs


class EventLoader(QObject):
    setReady = pyqtSignal(object, int)           # EventSet, generation
    overlayReady = pyqtSignal(object, int)       # Overlay, generation
    progress = pyqtSignal(str)
    failed = pyqtSignal(str)

    def __init__(self, cache: Optional[evs.StoreCache] = None, parent=None):
        super().__init__(parent)
        self.cache = cache if cache is not None else evs.StoreCache()
        self.gen = 0
        self._lock = threading.Lock()
        self._busy = False
        self._pending: Optional[tuple] = None
        self._es: Optional[evs.EventSet] = None
        self._es_json = None
        self._sels: Optional[list] = None
        self._sels_key = None
        self.n_px = 800
        self.max_traces: Optional[int] = None

    @property
    def busy(self) -> bool:
        return self._busy

    def request(self, query: evs.EventQuery, variables: list) -> int:
        with self._lock:
            self.gen += 1
            job = (self.gen, query, list(variables))
            if self._busy:
                self._pending = job
                return self.gen
            self._busy = True
        threading.Thread(target=self._run, args=(job,), name="wavi-events", daemon=True).start()
        return self.gen

    def _emit(self, sig, *args) -> None:
        try:
            sig.emit(*args)
        except RuntimeError:                       # the window is gone
            pass

    def _run(self, job) -> None:
        while job is not None:
            gen, q, variables = job
            try:
                self._work(gen, q, variables)
            except Exception as exc:  # noqa: BLE001 — reported in the panel
                traceback.print_exc()
                self._emit(self.failed, f"{type(exc).__name__}: {exc}")
            with self._lock:
                job, self._pending = self._pending, None
                if job is None:
                    self._busy = False

    def _stale(self, gen: int) -> bool:
        return gen != self.gen

    def _work(self, gen: int, q: evs.EventQuery, variables: list) -> None:
        qj = q.to_json()
        if qj != self._es_json or self._es is None:
            self._emit(self.progress, "reading the sidecars ...")
            es = evs.EventSet.from_query(q)
            self._es, self._es_json = es, qj
        es = self._es
        self._emit(self.setReady, es, gen)
        if self._stale(gen):
            return
        key = tuple((r.key, r.pinned, r.group) for r in es.events)
        if key != self._sels_key or self._sels is None:
            recs = list(dict.fromkeys(r.recording for r in es.events))
            todo = [r for r in recs if r not in self.cache]

            def prog(k, n, stem):
                if todo:
                    self._emit(self.progress, f"loading {min(k + 1, n)} of {n} recordings ({stem}) ...")
            sels = evs.event_selections(es, self.cache, progress=prog)
            self._sels, self._sels_key = sels, key
        if self._stale(gen):
            return
        self._emit(self.progress, f"computing {len(self._sels)} events' traces ...")
        ov = evs.overlay(self._sels, variables, self.n_px, self.max_traces)
        if self._stale(gen):
            return
        self._emit(self.overlayReady, ov, gen)
        self._emit(self.progress, f"{len(es)} events · {len(es.recordings)} recordings"
                   f" · traces in {ov.compute_s:.1f} s")
