#!/bin/bash
# Touch X 빠른 실시간 모니터링 (간소화된 출력)
# 사용법: ./monitor_touchx_fast.sh [joint_states|twist|position|velocity|effort]

CONTAINER_NAME="touchx_ros2"
TOPIC_TYPE=${1:-joint_states}

echo "=== Touch X 빠른 모니터링 ==="
echo ""

case $TOPIC_TYPE in
    joint_states|js)
        echo ">>> 조인트 상태 - 실시간 업데이트"
        docker exec -it ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        ros2 topic echo /geomagic_touch_x/joint_states | grep -E 'position:|velocity:|effort:' -A 6
        "
        ;;
    position|pos)
        echo ">>> Position 값만 - 실시간"
        docker exec -it ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        ros2 topic echo /geomagic_touch_x/joint_states | grep -A 6 'position:'
        "
        ;;
    velocity|vel)
        echo ">>> Velocity 값만 - 실시간"
        docker exec -it ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        ros2 topic echo /geomagic_touch_x/joint_states | grep -A 6 'velocity:'
        "
        ;;
    effort|torque)
        echo ">>> Effort (Torque) 값만 - 실시간"
        docker exec -it ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        ros2 topic echo /geomagic_touch_x/joint_states | grep -A 6 'effort:'
        "
        ;;
    twist|tw)
        echo ">>> Twist (속도) - 실시간"
        docker exec -it ${CONTAINER_NAME} bash -c "
        cd /home/ros/touchx_ws
        source /opt/ros/foxy/setup.bash
        source install/setup.bash 2>/dev/null || true
        ros2 topic echo /geomagic_touch_x/twist | grep -A 10 'twist:'
        "
        ;;
    *)
        echo "사용법: $0 [joint_states|position|velocity|effort|twist]"
        echo ""
        echo "옵션:"
        echo "  joint_states, js  - 전체 조인트 상태"
        echo "  position, pos     - Position 값만"
        echo "  velocity, vel     - Velocity 값만"
        echo "  effort, torque    - Effort 값만"
        echo "  twist, tw         - 속도 정보"
        ;;
esac

