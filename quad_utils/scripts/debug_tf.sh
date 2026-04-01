#!/bin/bash
# Run this on the robot while robot_driver is running
# Usage: bash debug_tf.sh

LOG="/root/ros2_ws/debug_tf.log"
echo "Logging to $LOG"
exec > >(tee "$LOG") 2>&1

echo "=== 1. Joint state publish rate ==="
timeout 5 ros2 topic hz /robot_1/state/joints 2>&1 | tail -3

echo ""
echo "=== 2. TF publish rate ==="
timeout 5 ros2 topic hz /tf 2>&1 | tail -3

echo ""
echo "=== 3. TF frames (5 messages) ==="
timeout 3 ros2 topic echo /tf -n 5 2>&1

echo ""
echo "=== 4. TF static frames ==="
timeout 3 ros2 topic echo /tf_static -n 5 2>&1

echo ""
echo "=== 5. Joint state sample ==="
ros2 topic echo /robot_1/state/joints --once 2>&1

echo ""
echo "=== 6. robot_state_publisher node info ==="
ros2 node info /robot_1/robot_state_publisher 2>&1

echo ""
echo "=== 7. robot_state_publisher params ==="
ros2 param get /robot_1/robot_state_publisher publish_frequency 2>&1
ros2 param get /robot_1/robot_state_publisher ignore_timestamp 2>&1
ros2 param get /robot_1/robot_state_publisher robot_description 2>&1 | head -10

echo ""
echo "=== 8. All topics ==="
ros2 topic list 2>&1

echo ""
echo "=== 9. System time vs joint state stamp ==="
echo "System time: $(date +%s.%N)"
ros2 topic echo /robot_1/state/joints --once 2>&1 | grep -A2 "stamp:"

echo ""
echo "=== 10. TF echo body->hip0 ==="
timeout 3 ros2 run tf2_ros tf2_echo body hip0 2>&1

echo ""
echo "=== 11. TF echo base_link->body ==="
timeout 3 ros2 run tf2_ros tf2_echo base_link body 2>&1

echo ""
echo "=== 12. URDF joint names from robot_description param ==="
ros2 param get /robot_1/robot_state_publisher robot_description 2>&1 | grep -oP 'joint name="\K[^"]+' | head -20

echo ""
echo "=== Done. Log saved to $LOG ==="
