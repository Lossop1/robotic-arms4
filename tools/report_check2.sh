set +e
ls -lt ~/runtime/reports/ 2>/dev/null | head -5
echo
F=$(ls -t ~/runtime/reports/orangepi-current-bench-*.json 2>/dev/null | head -1)
echo "目标文件: $F"
[ -z "$F" ] && exit 0
echo
echo "=== 键存在性 ==="
for k in shutdown_stop_criterion motion axes; do
  echo -n "  $k: "
  jq -e "has(\"$k\")" "$F" >/dev/null 2>&1 && echo yes || echo "no/err"
done
echo
echo "=== 停机判据（截断）==="
jq -c '.shutdown_stop_criterion' "$F" 2>&1 | head -c 800
echo
echo "=== 运动摘要（截断）==="
jq -c '.motion | {profile: .motion_profile_id, completed: .completed, started: .started,
                  axes: [.axes[]? | {axis_id, target, actual, error_counts, max_error_counts}]}' "$F" 2>&1 | head -c 900
