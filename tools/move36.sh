set +e
BIN=~/ethercat-master/build/apps/master/emaster-master
CFG=~/bench-runtime-lz
DEP=orangepi-current-bench
LOG=/tmp/arm-move.log
CSV=/tmp/arm-move.csv
rm -f /tmp/emaster-*.sock $LOG $CSV

echo "=== 启动（部署 $DEP 带 +36° 运动档）==="
sudo -n setsid nohup timeout 600 $BIN --config-dir $CFG --deployment $DEP > $LOG 2>&1 < /dev/null &
for i in $(seq 1 60); do ls /tmp/emaster-*.sock >/dev/null 2>&1 && break; sleep 0.5; done
SOCK=$(ls /tmp/emaster-*.sock 2>/dev/null | head -1)
echo "socket=$SOCK"
[ -z "$SOCK" ] && { tail -20 $LOG; exit 1; }

python3 - "$SOCK" "$CSV" <<'PY'
import socket, sys, time

sock, out = sys.argv[1], sys.argv[2]
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(sock); s.settimeout(3)

def cmd(c):
    s.sendall((c + "\n").encode())
    data = b""; t0 = time.time()
    while time.time() - t0 < 3:
        try:
            ch = s.recv(8192)
        except socket.timeout:
            break
        if not ch:
            break
        data += ch
        if data.endswith(b"\n"):
            break
    return data.decode("utf-8", "replace").strip()

def fields(line):
    if not line.startswith("OK|"):
        return None
    d = {}
    head, *parts = line.split("|")
    for p in parts:
        if p.startswith("a1:"):
            for kv in p[3:].split(","):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    d[k] = v
        elif "=" in p and not p.startswith("a1"):
            k, v = p.split("=", 1)
            d.setdefault(k, v)
    return d

print("=== 预读 ===")
st = fields(cmd("status"))
print("  state=%s enabled=%s pos=%s mode=%s" % (
    st.get("state"), st.get("enabled"), st.get("pos"), st.get("mode")))

rows = []
t0 = time.time()
while time.time() - t0 < 9.0:
    ts = time.time() - t0
    st = fields(cmd("status"))
    if st:
        rows.append((ts, st.get("pos"), st.get("vel"), st.get("torque"),
                     st.get("planned"), st.get("target_pos"), st.get("state"),
                     st.get("enabled"), st.get("wkc"), st.get("mode"), st.get("err")))
    time.sleep(0.02)

with open(out, "w") as f:
    f.write("t_s,pos,vel,torque,planned,target,state,enabled,wkc,mode,err\n")
    for r in rows:
        f.write(",".join("" if v is None else str(v) for v in r) + "\n")
print(f"=== 采集 {len(rows)} 行 -> {out} ===")

# 摘要
def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None

pos = [(r[0], num(r[1])) for r in rows if num(r[1]) is not None]
vel = [num(r[2]) for r in rows if num(r[2]) is not None]
tor = [num(r[3]) for r in rows if num(r[3]) is not None]
err = [(num(r[1]) - num(r[4])) for r in rows
       if num(r[1]) is not None and num(r[4]) is not None]

CPR = 16384 * 28
if pos:
    p0, p1 = pos[0][1], pos[-1][1]
    d = p1 - p0
    print("位置：%.0f -> %.0f counts，位移 %.0f counts = %.3f°" % (p0, p1, d, d / CPR * 360))
if vel:
    print("速度峰值 %.0f counts/s = %.2f °/s" % (max(vel, key=abs), max(vel, key=abs) / CPR * 360))
if tor:
    print("力矩 per-mille：min %d / max %d / 均值 %.1f（额定百分比）" % (min(tor), max(tor), sum(tor) / len(tor)))
if err:
    print("跟随误差峰值 %.0f counts = %.3f°" % (max(err, key=abs), abs(max(err, key=abs)) / CPR * 360))
print("状态集合:", sorted({r[6] for r in rows}), " enabled:", sorted({r[7] for r in rows}))
s.close()
PY

echo
echo "=== 停机 ==="
PID=$(pgrep -f 'emaster-master --config-dir' | head -1)
[ -n "$PID" ] && sudo -n kill -INT $PID
sleep 6
echo "=== 日志里的运动过程 ==="
grep -E 'MOTION|运动|完[t成]|MODE|target|完成' $LOG | tail -12
echo
echo "=== 日志尾部 ==="
tail -6 $LOG
