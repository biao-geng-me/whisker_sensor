"""A fake ``ssh`` for the sync tests: placed first on PATH (as ``ssh``, with a
shebang to this Python), it runs the remote command locally with ``sh -c``, as
sshd runs it in the login shell, so the remote root is just a temp dir.

``FAKE_SSH_MODE`` picks a failure: ``timeout`` / ``auth`` / ``hostkey`` /
``resolve`` (ssh's own messages, exit 255), ``drop`` (an upload's connection
dies halfway: half of stdin is passed, then the remote side is killed as sshd's
hang-up would, exit 255), ``drop_after:N`` (the same on the Nth upload, the
earlier ones succeed). ``FAKE_SSH_LOG``: one JSON line per call (argv).
"""

import json
import os
import signal
import subprocess
import sys

MSG = {
    "timeout": "ssh: connect to host {h} port 22: Connection timed out",
    "auth": "{h}: Permission denied (publickey,gssapi-with-mic).",
    "hostkey": "Host key verification failed.",
    "resolve": "ssh: Could not resolve hostname {h}: Name or service not known",
}


def main() -> int:
    argv = sys.argv[1:]
    log = os.environ.get("FAKE_SSH_LOG")
    if log:
        with open(log, "a") as f:
            f.write(json.dumps(argv) + "\n")
    i = 0
    while i < len(argv) and argv[i].startswith("-"):
        i += 2 if argv[i] in ("-o", "-p", "-i", "-l", "-F", "-J") else 1
    host, cmd = argv[i], " ".join(argv[i + 1:])
    mode = os.environ.get("FAKE_SSH_MODE", "")
    if mode in MSG:
        sys.stderr.write(MSG[mode].format(h=host) + "\n")
        return 255
    upload = "cat > " in cmd
    drop = False
    if upload and mode == "drop":
        drop = True
    elif upload and mode.startswith("drop_after:"):
        cnt = os.environ["FAKE_SSH_COUNT"]
        n = int(open(cnt).read() or 0) + 1 if os.path.exists(cnt) else 1
        open(cnt, "w").write(str(n))
        drop = n >= int(mode.split(":")[1])
    if not drop:
        return subprocess.run(["/bin/sh", "-c", cmd], stdin=sys.stdin.buffer).returncode
    data = sys.stdin.buffer.read()
    p = subprocess.Popen(["/bin/sh", "-c", cmd], stdin=subprocess.PIPE, start_new_session=True)
    p.stdin.write(data[: len(data) // 2])
    p.stdin.flush()
    import time
    time.sleep(0.3)
    os.killpg(p.pid, signal.SIGKILL)          # sshd hangs up the session
    p.wait()
    sys.stderr.write("client_loop: send disconnect: Broken pipe\n")
    return 255


if __name__ == "__main__":
    sys.exit(main())
