#!/bin/bash
# 자동 설치 및 설정 스크립트

set -e

echo "========================================="
echo "Geomagic Touch X + ROS 2 Foxy Setup"
echo "========================================="
echo ""

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Docker 확인
if ! command -v docker &> /dev/null; then
    echo -e "${RED}❌ Docker가 설치되어 있지 않습니다.${NC}"
    echo "Docker 설치: https://docs.docker.com/get-docker/"
    exit 1
fi

echo -e "${GREEN}✅ Docker 확인 완료${NC}"

# X11 확인
if [ -z "$DISPLAY" ]; then
    echo -e "${YELLOW}⚠️  DISPLAY 환경 변수가 설정되지 않았습니다.${NC}"
    echo "X11 forwarding이 필요합니다."
fi

# Docker 이미지 빌드
echo ""
echo "========================================="
echo "1단계: Docker 이미지 빌드"
echo "========================================="
echo ""

if [ -f "Dockerfile.touchx_ros2" ]; then
    echo "Docker 이미지를 빌드합니다... (시간이 오래 걸릴 수 있습니다)"
    docker build -f Dockerfile.touchx_ros2 -t touchx_ros2:latest .
    echo -e "${GREEN}✅ Docker 이미지 빌드 완료${NC}"
else
    echo -e "${RED}❌ Dockerfile.touchx_ros2를 찾을 수 없습니다.${NC}"
    exit 1
fi

# X11 접근 권한 설정
echo ""
echo "========================================="
echo "2단계: X11 접근 권한 설정"
echo "========================================="
xhost +local:docker 2>/dev/null || true
echo -e "${GREEN}✅ X11 접근 권한 설정 완료${NC}"

# Docker 컨테이너 생성
echo ""
echo "========================================="
echo "3단계: Docker 컨테이너 생성"
echo "========================================="

# 기존 컨테이너가 있으면 제거
if docker ps -a | grep -q touchx_ros2; then
    echo "기존 컨테이너를 제거합니다..."
    docker stop touchx_ros2 2>/dev/null || true
    docker rm touchx_ros2 2>/dev/null || true
fi

echo "새 컨테이너를 생성합니다..."
docker run -d --name touchx_ros2 \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  touchx_ros2:latest tail -f /dev/null

echo -e "${GREEN}✅ Docker 컨테이너 생성 완료${NC}"

# USB 디바이스 확인
echo ""
echo "========================================="
echo "4단계: USB 디바이스 확인"
echo "========================================="

if ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo "USB 디바이스를 찾았습니다:"
    ls -la /dev/ttyACM*
    echo ""
    echo "USB 권한을 설정합니다..."
    sudo chmod 666 /dev/ttyACM* 2>/dev/null || echo "권한 설정 실패 (나중에 수동으로 설정 필요)"
    echo -e "${GREEN}✅ USB 디바이스 확인 완료${NC}"
else
    echo -e "${YELLOW}⚠️  USB 디바이스를 찾을 수 없습니다.${NC}"
    echo "Touch X 디바이스가 연결되어 있는지 확인하세요."
fi

# 완료 메시지
echo ""
echo "========================================="
echo "✅ 설치 완료!"
echo "========================================="
echo ""
echo "다음 단계:"
echo ""
echo "1. Touch X 디바이스 설정:"
echo "   docker exec -it touchx_ros2 bash"
echo "   export GTDD_HOME=/usr/share/3DSystems"
echo "   export LD_LIBRARY_PATH=/opt/OpenHaptics/Developer/3.4-0/lib64:\$LD_LIBRARY_PATH"
echo "   export DISPLAY=:0"
echo "   sudo chmod 666 /dev/ttyACM0"
echo "   Touch_Setup"
echo ""
echo "2. ROS 2 드라이버 실행:"
echo "   ./start_rviz_final.sh"
echo ""
echo "자세한 내용은 README.md를 참조하세요."

