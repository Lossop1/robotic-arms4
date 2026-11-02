# -*- coding: utf-8 -*-
"""把本地脚本推到香橙派并执行（台架用）。"""
from __future__ import annotations

import sys
from pathlib import Path

from opssh import connect

ROOT = Path(__file__).resolve().parents[1]


def main():
    local = Path(sys.argv[1])
    remote = sys.argv[2]
    cmd = sys.argv[3] if len(sys.argv) > 3 else ""
    c = connect()
    try:
        sftp = c.open_sftp()
        sftp.put(str(local), remote)
        sftp.close()
        print(f"put {local.name} -> {remote}")
        if cmd:
            stdin, stdout, stderr = c.exec_command(cmd, timeout=900)
            out = stdout.read().decode("utf-8", "replace")
            err = stderr.read().decode("utf-8", "replace")
            rc = stdout.channel.recv_exit_status()
            print(f"[rc={rc}]")
            if out.strip():
                print(out.rstrip())
            if err.strip():
                print("[stderr]\n" + err.rstrip())
            return rc
    finally:
        c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
