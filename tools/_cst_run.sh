set +e
cd ~/ethercat-master
cp -a ~/bench-runtime-lz ~/bench-runtime-lz.bak.$(date +%H%M%S) 2>/dev/null
echo "=== 重编译扁平原文件 ==="
python3 tools/gen/emaster_config.py build --config ~/bench-config-lz --out ~/bench-runtime-lz 2>&1 | tail -5
echo "rc=$?"
grep -c 'orangepi-bench-lz-cst' ~/bench-runtime-lz/deployments.flat

DEP=orangepi-bench-lz-cst
BIN=~/ethercat-master/build/apps/master/emaster-master
rm -f /tmp/emaster-*.sock /tmp/obs_cst.csv /tmp/arm-cst.log
sudo -n env EMASTER_SETPOINT=1 EMASTER_OBSERVATION=1 setsid nohup timeout 200 $BIN \
  --config-dir ~/bench-runtime-lz --deployment $DEP > /tmp/arm-cst.log 2>&1 < /dev/null &
for i in $(seq 1 80); do [ -S /tmp/emaster-$DEP-sp.sock ] && break; sleep 0.5; done
ls -l /tmp/emaster-$DEP*.sock 2>/dev/null || { echo "CST 部署没起来"; tail -12 /tmp/arm-cst.log; exit 1; }
sleep 1
grep -iE 'mode|模式|CST' /tmp/arm-cst.log | head -3
python3 -c "
import socket,time
s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM); s.settimeout(2); s.connect('/tmp/emaster-$DEP.sock')
s.sendall(b'status\n'); d=b''
while not d.endswith(b'\n'): d+=s.recv(8192)
print([x for x in d.decode().split('|') if x.startswith('a1:')][0][:160])
"

nohup python3 /tmp/obs_record.py --deployment $DEP --seconds 30 --out /tmp/obs_cst.csv > /tmp/obs3.log 2>&1 &
OBS=$!
sleep 1
python3 /tmp/cst_ramp.py --deployment $DEP --amp 250 --ramp-s 6 --limit-deg 3 2>&1 | tail -4
echo "ramp rc=$?"
wait $OBS
cat /tmp/obs3.log
PID=$(pgrep -f "emaster-master --config-dir" | head -1)
[ -n "$PID" ] && sudo -n kill -INT $PID
sleep 4
wc -l /tmp/obs_cst.csv
