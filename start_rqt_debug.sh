#!/bin/bash
# rqt로 Touch X 디버깅 정보 확인

CONTAINER_NAME="touchx_ros2"

echo "=== rqt GUI 시작 (Touch X 디버깅) ==="

# X11 접근 권한 설정
xhost +local:docker 2>/dev/null || true

# rqt 실행
docker exec -it ${CONTAINER_NAME} bash -c "
export GTDD_HOME=/usr/share/3DSystems
export LD_LIBRARY_PATH=/opt/OpenHaptics/Developer/3.4-0/lib64:\$LD_LIBRARY_PATH
export DISPLAY=:0
cd /home/ros/touchx_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
rqt --perspective-file /opt/ros/foxy/share/rqt_gui_cpp/package.xml 2>/dev/null || rqt
"

echo ""
echo "rqt 창에서:"
echo "1. Plugins → Topics → Topic Monitor 선택"
echo "2. 왼쪽에서 '/geomagic_touch_x/joint_states' 토픽 찾기"
echo "3. 토픽을 선택하면 오른쪽에 position, velocity, effort 값이 실시간으로 표시됨"

