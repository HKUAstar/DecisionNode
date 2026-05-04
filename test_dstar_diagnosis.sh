#!/bin/bash

# 诊断脚本：测试dstar_status消息流和CheckArrived黑板同步
# 此脚本启动base_move_test环境并测试dstar状态传播

set -e

WSL_PATH="/mnt/d/decision_ws"

echo "=========================================="
echo "D* Status Diagnostic Test"
echo "=========================================="

# 启动ROS和base_move_test launch
echo "[1] Starting base_move_test launch..."
wsl bash -c "
cd $WSL_PATH
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

# Kill any existing roscore processes
pkill -f roscore || true
sleep 1

# Start roscore in the background
roscore &
ROSCORE_PID=\$!
sleep 2

# Launch base_move_test
echo '[INFO] Launching base_move_test...'
timeout 30 roslaunch decision_node base_move_test.launch

# Clean up
kill \$ROSCORE_PID 2>/dev/null || true
" &

BASE_LAUNCH_PID=$!

# Wait for launch to start
sleep 10

echo ""
echo "[2] Publishing dstar_status test messages..."
sleep 2

# Publish test messages
for i in 1 2 3; do
  echo "  - Publishing dstar_status = true (iteration $i)"
  wsl bash -c "
  cd $WSL_PATH
  source devel/setup.bash
  export ROS_MASTER_URI=http://localhost:11311
  rostopic pub -1 /dstar_status std_msgs/Bool '{data: true}'
  "
  sleep 3
  
  echo "  - Publishing dstar_status = false (iteration $i)"
  wsl bash -c "
  cd $WSL_PATH
  source devel/setup.bash
  export ROS_MASTER_URI=http://localhost:11311
  rostopic pub -1 /dstar_status std_msgs/Bool '{data: false}'
  "
  sleep 3
done

echo ""
echo "[3] Checking dstar_status topic..."
wsl bash -c "
cd $WSL_PATH
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
echo 'Recent messages on /dstar_status:'
timeout 5 rostopic echo /dstar_status 2>/dev/null | head -20 || true
"

echo ""
echo "=========================================="
echo "Test Complete"
echo "=========================================="

# Clean up
wait $BASE_LAUNCH_PID 2>/dev/null || true
pkill -f roscore || true
pkill -f strategy_node || true

