set +e
C=~/bench-config-lz
echo "=== 部署 orangepi-current-bench ==="
cat $C/deployments/orangepi_current_bench.json 2>/dev/null || ls $C/deployments | grep -i current

echo
echo "=== 运动档 output-positive-36deg ==="
cat $C/motion_profiles/orangepi_bench_output_positive_36deg.json 2>/dev/null | head -60

echo
echo "=== 该运动档的字段摘要 ==="
jq -c '{id: .motion_profile_id, kind: (.kind // .type), axes: (.axes|length?)}' $C/motion_profiles/orangepi_bench_output_positive_36deg.json 2>/dev/null
