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
        echo ">>> 조인트 상태 모니터링 (/geomagic_touch_x/joint_states)"
        docker exec -it ${CONTAINER_NAME} bash -c "source /opt/ros/foxy/setup.bash && ros2 topic echo /geomagic_touch_x/joint_states"
        ;;
    twist)
        echo ">>> 속도 정보 모니터링 (/geomagic_touch_x/twist)"
        docker exec -it ${CONTAINER_NAME} bash -c "source /opt/ros/foxy/setup.bash && ros2 topic echo /geomagic_touch_x/twist"
        ;;
    tf)
        echo ">>> TF 변환 모니터링 (touch_x_base -> touch_x_ee)"
        docker exec -it ${CONTAINER_NAME} bash -c "source /opt/ros/foxy/setup.bash && ros2 run tf2_ros tf2_echo touch_x_base touch_x_ee"
        ;;
    all|*)
        echo ">>> 모든 토픽 요약 정보"
        echo ""
        echo "1. 조인트 상태 (position, velocity, effort)"
        docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/foxy/setup.bash && timeout 3 ros2 topic echo /geomagic_touch_x/joint_states 2>&1 | head -20" &
        sleep 4
        echo ""
        echo "2. 속도 정보 (linear, angular velocity)"
        docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/foxy/setup.bash && timeout 3 ros2 topic echo /geomagic_touch_x/twist 2>&1 | head -15" &
        sleep 4
        echo ""
        echo "3. 토픽 주파수 확인"
        docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/foxy/setup.bash && timeout 5 ros2 topic hz /geomagic_touch_x/joint_states 2>&1"
        ;;
esac

