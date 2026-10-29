#!/bin/bash
# wait until both controllers are active (or 180 s), then show the interesting launch log lines
for i in $(seq 1 90); do
  grep -q "Configured and activated.*painting_controller" ~/gz_launch.log 2>/dev/null && break
  grep -qi "process has died" ~/gz_launch.log 2>/dev/null && break
  sleep 2
done
sed 's/\x1b\[[0-9;]*m//g' ~/gz_launch.log | grep -Ei "error|died|activated|spawn|entity|fail|warn" | grep -v "Material service\|SetVisibilityMask" | cut -c1-220 | tail -${1:-25}
