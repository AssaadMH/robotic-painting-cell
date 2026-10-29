#!/bin/bash
# wait for move_group to report ready, then show the interesting log lines
for i in $(seq 1 45); do
  grep -q "You can start planning now" ~/launch.log 2>/dev/null && break
  sleep 2
done
grep -E "rror|planning now|Failed|WARN|ready" ~/launch.log | cut -c1-230 | tail -${1:-20}
echo "--- processes:"; pgrep -fa "move_group|robot_state_pub" | cut -c1-70
