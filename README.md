# Geomagic Touch X + ROS 2 Foxy

USB-connected Geomagic Touch X haptic device를 ROS 2 Foxy와 연동하는 솔루션입니다.

## 📋 사전 요구사항

- Docker 설치 (버전 20.10 이상)
- Docker Compose (선택사항)
- USB 포트 접근 권한
- X11 forwarding (RViz2 GUI용)
- 최소 10GB 디스크 공간 (Docker 이미지용)

## 🚀 빠른 시작

### 방법 1: 자동 설치 스크립트 (권장)

```bash
git clone https://github.com/RACCOOONkim/touch-x-ros2.git
cd touch-x-ros2
chmod +x SETUP.sh
./SETUP.sh
```

이 스크립트는 다음을 자동으로 수행합니다:
- Docker 설치 확인
- Docker 이미지 빌드 (`touchx_ros2:latest`)
- X11 접근 권한 설정
- Docker 컨테이너 생성 및 시작

### 방법 2: 수동 설치

#### 1. Docker 이미지 빌드

```bash
git clone https://github.com/RACCOOONkim/touch-x-ros2.git
cd touch-x-ros2

# Docker 이미지 빌드 (시간이 오래 걸릴 수 있음, 약 10-20분)
docker build -f Dockerfile.touchx_ros2 -t touchx_ros2:latest .
```

#### 2. X11 접근 권한 설정

```bash
xhost +local:docker
```

#### 3. Docker 컨테이너 실행

```bash
docker run -d --name touchx_ros2 \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  touchx_ros2:latest tail -f /dev/null
```

**중요**: `--privileged`와 `--network host` 옵션은 USB 디바이스 접근과 X11 forwarding에 필요합니다.

## ⚙️ Touch X 디바이스 설정 (최초 1회)

Docker 컨테이너가 실행 중이어야 합니다.

```bash
docker exec -it touchx_ros2 bash -c "
export GTDD_HOME=/usr/share/3DSystems
export LD_LIBRARY_PATH=/opt/OpenHaptics/Developer/3.4-0/lib64:\$LD_LIBRARY_PATH
export DISPLAY=:0
sudo chmod 666 /dev/ttyACM0
Touch_Setup
"
```

## 🎮 실행

컨테이너가 실행 중이고 Touch X가 설정되었다면:

```bash
./start_rviz_final.sh
```

## 📊 제공되는 ROS 2 토픽

- `/geomagic_touch_x/joint_states` - 6개 조인트의 position, velocity, effort
- `/geomagic_touch_x/twist` - 엔드 이펙터 선속도 및 각속도
- `/tf` - `touch_x_base` → `touch_x_ee` 변환

## 📝 사용 방법

### 데이터 모니터링

```bash
./monitor_touchx.sh joint_states
```

### rqt GUI 사용

```bash
./start_rqt_debug.sh
```

## 📁 프로젝트 구조

```
TouchX-Repo/
├── README.md              # 이 파일
├── SETUP.sh              # 자동 설치 스크립트
├── Dockerfile.touchx_ros2 # Docker 이미지
├── start_rviz_final.sh   # 메인 실행 스크립트
├── monitor_touchx.sh     # 토픽 모니터링
├── start_rqt_debug.sh    # rqt 디버깅 GUI
├── 3ds-touch-openhaptics/ # 드라이버 설치 스크립트
└── geomagic_touch_x_ros2/  # ROS 2 드라이버 패키지
```

## 🔧 문제 해결

### Docker 이미지 빌드 실패

```bash
# 네트워크 연결 확인
ping google.com

# Docker 이미지 크기 확인 (약 3-4GB)
docker images touchx_ros2

# 빌드 캐시 지우고 다시 빌드
docker build --no-cache -f Dockerfile.touchx_ros2 -t touchx_ros2:latest .
```

### 컨테이너가 시작 안 됨

```bash
# 기존 컨테이너 확인 및 제거
docker ps -a | grep touchx_ros2
docker stop touchx_ros2 2>/dev/null
docker rm touchx_ros2 2>/dev/null

# 다시 생성
docker run -d --name touchx_ros2 \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  touchx_ros2:latest tail -f /dev/null
```

### USB 디바이스 인식 안 됨

```bash
# USB 디바이스 확인
ls -la /dev/ttyACM*

# 권한 설정
sudo chmod 666 /dev/ttyACM0

# 컨테이너에서도 확인
docker exec touchx_ros2 bash -c "ls -la /dev/ttyACM*"
```

### RViz2 GUI가 안 열림

```bash
# X11 접근 권한 확인
xhost +local:docker

# DISPLAY 환경 변수 확인
echo $DISPLAY

# 컨테이너가 host 네트워크 모드로 실행되었는지 확인
docker inspect touchx_ros2 | grep -i network
```

### Touch X 초기화 실패

```bash
# Touch_Diagnostic으로 연결 확인
docker exec -it touchx_ros2 bash -c "
export GTDD_HOME=/usr/share/3DSystems
export LD_LIBRARY_PATH=/opt/OpenHaptics/Developer/3.4-0/lib64:\$LD_LIBRARY_PATH
export DISPLAY=:0
sudo chmod 666 /dev/ttyACM0
Touch_Diagnostic
"

# 설정 파일 확인
docker exec touchx_ros2 bash -c "cat /usr/share/3DSystems/config/Default\ Device.config"
```

## 📚 라이센스

MIT License

