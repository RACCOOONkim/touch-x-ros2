#!/bin/bash
# Touch X 드라이버와 RViz2 GUI 실행 (최종 완성 버전 - host 네트워크 모드)

CONTAINER_NAME="touchx_ros2"

echo "=== Touch X 드라이버 + RViz2 GUI 시작 (최종 버전) ==="

# X11 접근 권한 설정
xhost +local:docker 2>/dev/null || true

# 컨테이너가 host 네트워크 모드로 실행되고 있는지 확인
if ! docker inspect ${CONTAINER_NAME} 2>/dev/null | grep -q "NetworkMode.*host"; then
    echo "⚠️  경고: 컨테이너가 host 네트워크 모드로 실행되지 않았습니다."
    echo "RViz2 GUI를 위해 host 네트워크 모드가 필요합니다."
    echo ""
    echo "컨테이너 재생성 방법:"
    echo "docker stop ${CONTAINER_NAME}"
    echo "docker rm ${CONTAINER_NAME}"
    echo "xhost +local:docker"
    echo "docker run -d --name ${CONTAINER_NAME} --privileged --network host -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e DISPLAY=\$DISPLAY touchx_ros2:backup tail -f /dev/null"
    exit 1
fi

# 기존 프로세스 정리
docker exec ${CONTAINER_NAME} bash -c "pkill -f device_driver; pkill -f ros2; pkill -f rviz2; sleep 1" 2>/dev/null || true

# 드라이버와 RViz2 함께 실행
docker exec -d ${CONTAINER_NAME} bash -c "
source /opt/ros/foxy/setup.bash
export GTDD_HOME=/usr/share/3DSystems
export LD_LIBRARY_PATH=/opt/OpenHaptics/Developer/3.4-0/lib64:/opt/ros/foxy/lib:/opt/ros/foxy/opt/rviz_ogre_vendor/lib:/opt/ros/foxy/opt/yaml_cpp_vendor/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:\$LD_LIBRARY_PATH
export DISPLAY=:0
sudo chmod 666 /dev/ttyACM0
cd /home/ros/touchx_ws
source install/setup.bash
ros2 launch geomagic_touch_x device_driver.launch.py rviz:=true debug:=true 2>&1 > /tmp/touchx_rviz_final.log
"

echo "드라이버 및 RViz2 시작 중... (8초 대기)"
sleep 8

# 상태 확인
echo ""
echo "=== 실행 상태 확인 ==="
if docker exec ${CONTAINER_NAME} bash -c "ps aux | grep device_driver | grep -v grep | grep -v defunct"; then
    echo "✅ 드라이버: 실행 중"
else
    echo "❌ 드라이버: 실행 안 됨"
fi

if docker exec ${CONTAINER_NAME} bash -c "ps aux | grep rviz2 | grep -v grep"; then
    echo "✅ RViz2: 실행 중"
    echo ""
    echo "🎉🎉🎉 성공! RViz2 GUI가 실행되었습니다! 🎉🎉🎉"
    echo ""
    echo "RViz2 창에서 확인할 수 있는 것:"
    echo "1. TF 변환: touch_x_base → touch_x_ee 축과 화살표"
    echo "2. Touch X를 움직이면 엔드 이펙터 위치가 실시간으로 업데이트됨"
    echo "3. 좌측 Displays 패널에서 추가 시각화 요소 추가 가능"
    echo ""
    echo "👉 Touch X를 움직여보세요! RViz2에서 실시간으로 확인할 수 있습니다!"
else
    echo "❌ RViz2: 실행 실패"
    echo ""
    echo "로그 확인:"
    docker exec ${CONTAINER_NAME} bash -c "cat /tmp/touchx_rviz_final.log 2>/dev/null | tail -25"
fi

echo ""
echo "토픽 확인:"
docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/foxy/setup.bash && ros2 topic list | grep touch_x"

