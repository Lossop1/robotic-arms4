# -*- coding: utf-8 -*-
"""
香橙派只读侦察：SSH 进去跑几条命令，什么都不改。

用法:
  python bench/opssh.py "hostname; uname -a" "ip -br a"

多个参数按顺序执行。默认主机 orangepi@192.168.124.81。
"""
from __future__ import annotations

import sys
import time

import paramiko

HOST = "192.168.124.81"
USER = "orangepi"
PWD = "orangepi"


def connect(timeout=8):
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PWD, timeout=timeout,
              auth_timeout=timeout, banner_timeout=timeout,
              look_for_keys=False, allow_agent=False)
    return c


def run(client, cmd, timeout=25):
    t0 = time.time()
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    rc = stdout.channel.recv_exit_status()
    dt = time.time() - t0
    print(f"$ {cmd}   [{rc}, {dt:.1f}s]")
    if out.strip():
        print("\n".join("  " + l for l in out.rstrip().splitlines()))
    if err.strip():
        print("\n".join("  [err] " + l for l in err.rstrip().splitlines()))
    print()
    return out


def main():
    args = sys.argv[1:]
    if args and args[0] in ("--version", "-V"):
        print(f"opssh.py 0.1.0")
        return 0
    if args and args[0] in ("--help", "-h"):
        print("用法: opssh.py [--file <本地脚本> | <远端命令> ...]\n"
              "  --version  打印版本后退出\n"
              "  --file X   把本地脚本整段喂给远端 bash -s\n"
              "  不给参数   打印本说明")
        return 0
    if args and args[0] == "--file":
        # 把本地脚本整段喂给远端 bash -s，彻底绕开引号转义
        script = open(args[1], encoding="utf-8").read()
        try:
            c = connect()
        except Exception as e:
            print(f"连不上 {USER}@{HOST}: {type(e).__name__}: {e}")
            return 2
        try:
            stdin, stdout, stderr = c.exec_command("bash -s", timeout=900)
            stdin.write(script)
            stdin.channel.shutdown_write()
            out = stdout.read().decode("utf-8", "replace")
            err = stderr.read().decode("utf-8", "replace")
            rc = stdout.channel.recv_exit_status()
            print(f"# {args[1]}  [rc={rc}]")
            if out.strip():
                print(out.rstrip())
            if err.strip():
                print("[stderr]\n" + err.rstrip())
        finally:
            c.close()
        return 0

    cmds = args or ["hostname; uname -a; uptime"]
    try:
        c = connect()
    except Exception as e:
        print(f"连不上 {USER}@{HOST}: {type(e).__name__}: {e}")
        return 2
    try:
        for cmd in cmds:
            run(c, cmd)
    finally:
        c.close()
    return 0


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
