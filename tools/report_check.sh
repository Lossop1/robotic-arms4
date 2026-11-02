set +e
R=~/runtime/reports/orangepi-current-bench-latest.json
echo "=== 报告：运动与停机相关 ==="
jq -c '{status: .result.status_code, cycles: .result.cycle_count,
        wkc: [.result.expected_wkc, .result.actual_wkc],
        deadline_missed: .result.cycle_deadline_missed}' $R 2>/dev/null

echo
echo "=== motion 段 ==="
jq -c '.motion' $R 2>/dev/null | head -c 1200
echo
echo "=== 停机判据 ==="
jq -c '.shutdown_stop_criterion' $R 2>/dev/null | head -c 900
echo
echo "=== 轴的运动字段 ==="
jq -c '.axes[0] | {motion_final_position: .runtime.motion_final_position,
                  initial: .runtime.initial_actual_position,
                  max_following_error: .runtime.max_observed_following_error_counts,
                  target_reached: .runtime.target_reached,
                  final_error_code: .runtime.runtime_error_code}' $R 2>/dev/null
