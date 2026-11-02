set +e
sudo -n ls -lt /home/orangepi/runtime/reports/ 2>/dev/null | head -4
echo
F=$(sudo -n ls -t /home/orangepi/runtime/reports/orangepi-current-bench-*.json 2>/dev/null | head -1)
echo "目标: $F"
[ -z "$F" ] && exit 0
echo
echo "=== 停机判据 ==="
sudo -n jq -c '.shutdown_stop_criterion' "$F" 2>&1 | head -c 700
echo
echo
echo "=== 运动摘要 ==="
sudo -n jq -c '.motion | {profile_id, started, completed,
    axes: [.axes[]? | {axis_id, target_position, actual_position,
                       endpoint_error_counts, max_following_error_counts}]}' "$F" 2>&1 | head -c 900
echo
echo
echo "=== 轴运行统计 ==="
sudo -n jq -c '.axes[0].runtime | {initial: .initial_actual_position,
    final: .motion_final_position, max_fe: .max_observed_following_error_counts,
    target_reached, err: .runtime_error_code}' "$F" 2>&1 | head -c 400
