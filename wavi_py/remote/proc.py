"""Running OpenSSH from the app on Windows, macOS and Linux.

Everything goes through ``ssh <host> '<command>'`` (no scp): an upload pipes the
local file into ``cat > <file>.part`` on the cluster. That keeps one tool and
sidesteps scp's two traps: a local path with a drive letter (``C:\\data\\x``
reads as host ``C``) and a remote path that the legacy protocol hands to the
remote shell while the newer SFTP one does not, so the right quoting depends
on the OpenSSH version. Here the local file is only ever an open handle, and
the remote command is one string quoted by ``shlex.quote`` for the cluster's
POSIX shell, whatever the client's OS.

- ``find_ssh``: ``shutil.which``; on Windows also the OpenSSH Client feature's
  folder (``System32\\OpenSSH``, and ``Sysnative`` for a 32-bit Python, which
  sees System32 redirected to SysWOW64 where there is no ssh);
- ``popen_kwargs``: no shell, no console window on Windows (``CREATE_NO_WINDOW``);
- ``ssh_argv``: BatchMode (never a prompt: no password, no host-key question),
  a 5 s connect timeout and keepalives, so a dead link fails in ~30 s rather
  than waiting for the call's own timeout;
- ``run``: one call with a timeout, cancellable from another thread (the GUI
  closing), stdin optionally an open file or bytes. An open file is fed
  through a pipe in ``FEED_CHUNK`` pieces by a writer thread, so an upload can
  report its bytes (``on_bytes``, at most ``EMIT_HZ`` times a second, plus once
  at the end); the count is what went into ssh's stdin, i.e. ahead of the wire
  by ssh's own buffers (a few MB), which is why the caller shows "verifying"
  while it waits for the remote check after the last byte;
- ``classify``: why a call failed, in words, from the exit code and stderr;
  ``offline`` says whether it was the network (retry later) or the setup.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import IO, Callable, Optional, Sequence, Union

CONNECT_TIMEOUT_S = 5
SSH_OPTS = ("-o", "BatchMode=yes", "-o", f"ConnectTimeout={CONNECT_TIMEOUT_S}",
            "-o", "ServerAliveInterval=10", "-o", "ServerAliveCountMax=3")


FEED_CHUNK = 1 << 20      # bytes per write into ssh's stdin
EMIT_HZ = 4.0             # byte callbacks per second, at most


class SshMissing(RuntimeError):
    pass


def windows_ssh_candidates(environ=None) -> list[str]:
    """Where the Windows OpenSSH Client optional feature puts ssh.exe."""
    env = os.environ if environ is None else environ
    root = env.get("SystemRoot") or env.get("WINDIR") or r"C:\Windows"
    return [rf"{root}\System32\OpenSSH\ssh.exe", rf"{root}\Sysnative\OpenSSH\ssh.exe"]


def find_ssh(platform: Optional[str] = None, environ=None) -> str:
    p = shutil.which("ssh")
    if p:
        return p
    if (platform or sys.platform) == "win32":
        for c in windows_ssh_candidates(environ):
            if os.path.isfile(c):
                return c
    raise SshMissing("no ssh on PATH (Windows: add the OpenSSH Client optional feature)")


def popen_kwargs(platform: Optional[str] = None) -> dict:
    """subprocess options: never a shell; on Windows no console window flashes
    up for each call of a GUI app (pythonw has none to share)."""
    kw: dict = {"shell": False}
    if (platform or sys.platform) == "win32":
        kw["creationflags"] = getattr(subprocess, "CREATE_NO_WINDOW", 0x08000000)
    return kw


def ssh_argv(ssh: str, host: str, remote_cmd: str) -> list[str]:
    """``ssh <opts> host cmd``. A host starting with ``-`` would read as an
    option, so it is refused."""
    if not host or host.startswith("-"):
        raise ValueError(f"bad host {host!r}")
    return [ssh, *SSH_OPTS, host, remote_cmd]


class Throttle:
    """Calls ``fn(done, total)`` at most ``hz`` times a second; ``final=True``
    always goes through (the 100 % that ends a file)."""

    def __init__(self, fn, hz: float = EMIT_HZ):
        self._fn, self._dt, self._last = fn, 1.0 / hz, -1e9

    def __call__(self, done: int, total: int, final: bool = False) -> None:
        now = time.monotonic()
        if final or now - self._last >= self._dt:
            self._last = now
            try:
                self._fn(done, total)
            except Exception:  # noqa: BLE001 — a display callback never breaks a transfer
                pass


def _feed(src: IO[bytes], dst: IO[bytes], on_bytes: Optional[Callable[[int, int], None]],
          total: int) -> None:
    """The writer thread: ``src`` into the pipe ``dst`` in chunks, then close it
    (the remote ``cat`` sees EOF). A dead process (killed by the timeout or a
    cancel, or ssh exiting on a dropped link) ends it with a broken pipe."""
    tick = Throttle(on_bytes) if on_bytes else None
    done = 0
    try:
        while True:
            b = src.read(FEED_CHUNK)
            if not b:
                break
            dst.write(b)
            done += len(b)
            if tick:
                tick(done, total)
        dst.flush()
        if tick:
            tick(done, total, final=True)
    except (OSError, ValueError):             # BrokenPipe / closed: the process is gone
        pass
    finally:
        try:
            dst.close()
        except (OSError, ValueError):
            pass


@dataclass
class Result:
    rc: int                  # exit code; -1: timed out, -2: cancelled, -3: no ssh
    out: bytes
    err: str

    @property
    def ok(self) -> bool:
        return self.rc == 0


class Runner:
    """Runs the calls; ``cancel()`` (any thread) kills the one running and makes
    every later one fail at once."""

    def __init__(self, ssh: Optional[str] = None):
        self._ssh = ssh
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._cancelled = False

    def ssh(self) -> str:
        if self._ssh is None:
            self._ssh = find_ssh()
        return self._ssh

    def cancel(self) -> None:
        with self._lock:
            self._cancelled = True
            if self._proc is not None:
                try:
                    self._proc.kill()
                except OSError:
                    pass

    def run(self, host: str, remote_cmd: str, timeout: float,
            stdin: Union[IO[bytes], bytes, None] = None,
            on_bytes: Optional[Callable[[int, int], None]] = None) -> Result:
        try:
            argv = ssh_argv(self.ssh(), host, remote_cmd)
        except SshMissing as exc:
            return Result(-3, b"", str(exc))
        return self.run_argv(argv, timeout, stdin, on_bytes)

    def run_argv(self, argv: Sequence[str], timeout: float,
                 stdin: Union[IO[bytes], bytes, None] = None,
                 on_bytes: Optional[Callable[[int, int], None]] = None) -> Result:
        """stdin: bytes (handed to ``communicate``), an open binary file (fed by
        ``_feed`` from a thread; ``on_bytes(done, total)`` reports it), or None.
        The timeout counts from the start, the whole transfer included."""
        with self._lock:
            if self._cancelled:
                return Result(-2, b"", "cancelled")
            data = stdin if isinstance(stdin, (bytes, bytearray)) else None
            src = subprocess.PIPE if stdin is not None else subprocess.DEVNULL
            try:
                p = subprocess.Popen(list(argv), stdin=src, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, **popen_kwargs())
            except FileNotFoundError:
                return Result(-3, b"", f"no ssh at {argv[0]}")
            self._proc = p
        feeder = None
        if stdin is not None and data is None:
            pipe, p.stdin = p.stdin, None         # communicate() must not touch it
            try:
                total = os.fstat(stdin.fileno()).st_size - stdin.tell()
            except (OSError, AttributeError, ValueError):
                total = 0
            feeder = threading.Thread(target=_feed, args=(stdin, pipe, on_bytes, total),
                                      name="wavi-ssh-feed", daemon=True)
            feeder.start()
        try:
            out, err = p.communicate(input=data, timeout=timeout)
            rc = p.returncode
        except subprocess.TimeoutExpired:
            p.kill()
            out, err = p.communicate()
            rc = -1
        finally:
            if feeder is not None:
                if p.poll() is None:              # an exception above: don't leave it
                    p.kill()
                feeder.join(10)
            with self._lock:
                self._proc = None
                if self._cancelled:
                    rc = -2
        return Result(rc, out, err.decode("utf-8", "replace"))

    def stream(self, host: str, remote_cmd: str, sink, timeout: float) -> Result:
        """Run ``remote_cmd`` and hand its stdout to ``sink(chunk)`` as it comes (a
        download); ``Result.out`` stays empty. Cancel kills it as ``run``'s."""
        try:
            argv = ssh_argv(self.ssh(), host, remote_cmd)
        except SshMissing as exc:
            return Result(-3, b"", str(exc))
        with self._lock:
            if self._cancelled:
                return Result(-2, b"", "cancelled")
            try:
                p = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, **popen_kwargs())
            except FileNotFoundError:
                return Result(-3, b"", f"no ssh at {argv[0]}")
            self._proc = p
        err: list = []
        reader = threading.Thread(target=lambda: err.append(p.stderr.read()), daemon=True)
        reader.start()
        timed_out = []

        def kill():
            timed_out.append(True)
            p.kill()
        timer = threading.Timer(timeout, kill)
        timer.start()
        try:
            while True:
                b = p.stdout.read(1 << 20)
                if not b:
                    break
                sink(b)
            rc = p.wait()
        except BaseException:
            p.kill()
            p.wait()
            raise
        finally:
            timer.cancel()
            reader.join(5)
            with self._lock:
                self._proc = None
                cancelled = self._cancelled
        rc = -2 if cancelled else -1 if timed_out else rc
        return Result(rc, b"", (err[0] if err else b"").decode("utf-8", "replace"))


# (stderr fragment, reason, offline): the first match wins
_PATTERNS = (
    ("host key verification failed", "host key unknown or changed: run `ssh {host} true`"
     " once in a terminal and accept it", False),
    ("remote host identification has changed", "host key changed (see ~/.ssh/known_hosts)",
     False),
    ("permission denied", "auth refused: no key loaded, or the key isn't authorized"
     " on the cluster", False),
    ("too many authentication failures", "auth refused (too many keys offered)", False),
    ("could not resolve hostname", "unknown host (no network, or a typo in the host)", True),
    ("name or service not known", "unknown host (no network, or a typo in the host)", True),
    ("no such host is known", "unknown host (no network, or a typo in the host)", True),
    ("timed out", "timeout (no network, VPN off, or the host is down)", True),
    ("connection refused", "connection refused (sshd down, or a wrong port)", True),
    ("no route to host", "no route to host (no network, VPN off)", True),
    ("network is unreachable", "network unreachable", True),
    ("connection reset", "connection reset", True),
    ("connection closed", "connection closed by the host", True),
    ("broken pipe", "connection lost", True),
    ("timeout, server", "connection lost (keepalive timeout)", True),
)


def classify(res: Result, host: str = "") -> tuple[str, bool]:
    """(a reason in words, offline?) for a failed call. ssh's own errors exit
    255; any other code is the remote command's."""
    if res.rc == -3:
        return res.err or "no ssh on PATH", False
    if res.rc == -2:
        return "cancelled", True
    if res.rc == -1:
        return "timeout (no reply)", True
    err = res.err.lower()
    if res.rc == 255:
        for frag, why, offline in _PATTERNS:
            if frag in err:
                return why.format(host=host or "<host>"), offline
        last = (res.err.strip().splitlines() or ["ssh failed"])[-1]
        return f"ssh: {last[:160]}", True
    last = (res.err.strip().splitlines() or [""])[-1]
    return f"remote command failed (exit {res.rc}){': ' + last[:160] if last else ''}", False
