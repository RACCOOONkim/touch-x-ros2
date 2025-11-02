#!/bin/bash
# Touch X 실시간 모니터링 스크립트
# 사용법: ./monitor_touchx.sh [joint_states|twist|tf|all]

CONTAINER_NAME="touchx_ros2"
TOPIC_TYPE=${1:-all}

echo "=== Geomagic Touch X ROS 2 모니터링 ==="
echo "Touch X를 움직이면 실시간으로 데이터가 업데이트됩니다."
echo "종료하려면 Ctrl+C를 누르세요."
echo ""

case $TOPIC_TYPE in
    joint_states)
        echo ">>> 조인트 상태 모니터링 (/geomagic_touch_x/joint_states) - 실시간"
        echo "Touch X를 움직이면 값이 실시간으로 업데이트됩니다..."
        echo ""
        docker exec -it ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        ros2 topic echo /geomagic_touch_x/joint_states
        "
        ;;
    twist)
        echo ">>> 속도 정보 모니터링 (/geomagic_touch_x/twist) - 실시간"
        echo "Touch X를 움직이면 값이 실시간으로 업데이트됩니다..."
        echo ""
        docker exec -it ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        ros2 topic echo /geomagic_touch_x/twist
        "
        ;;
    tf)
        echo ">>> TF 변환 모니터링 (touch_x_base -> touch_x_ee)"
        docker exec -it ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        ros2 run tf2_ros tf2_echo touch_x_base touch_x_ee
        "
        ;;
    hz|frequency)
        echo ">>> 토픽 발행 주파수 확인"
        docker exec -it ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        echo 'joint_states 주파수:'
        timeout 5 ros2 topic hz /geomagic_touch_x/joint_states 2>&1 || echo '발행 중이 아닙니다'
        echo ''
        echo 'twist 주파수:'
        timeout 5 ros2 topic hz /geomagic_touch_x/twist 2>&1 || echo '발행 중이 아닙니다'
        "
        ;;
    all|*)
        echo ">>> 모든 토픽 요약 정보 (최신 1개 메시지만)"
        echo ""
        echo "1. 조인트 상태 (최신 값)"
        docker exec ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        timeout 2 ros2 topic echo /geomagic_touch_x/joint_states 2>&1 | head -25 || echo '토픽 데이터 없음 (드라이버가 실행 중인지 확인하세요)'
        "
        echo ""
        echo "2. 속도 정보 (최신 값)"
        docker exec ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        timeout 2 ros2 topic echo /geomagic_touch_x/twist 2>&1 | head -20 || echo '토픽 데이터 없음'
        "
        echo ""
        echo "3. 토픽 주파수 확인"
        docker exec ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        timeout 5 ros2 topic hz /geomagic_touch_x/joint_states 2>&1 || echo '토픽 발행 중이 아닙니다'
        "
        ;;
esac

