# Geomagic Touch X + ROS 2 Foxy

USB-connected Geomagic Touch X haptic device를 ROS 2 Foxy와 연동하고 RViz2로 실시간 시각화하는 완전한 솔루션입니다.

## ✨ 주요 기능

- ✅ **Geomagic Touch X USB 디바이스 지원**
- ✅ **ROS 2 Foxy 통합** - 실시간 데이터 발행 (200Hz)
- ✅ **RViz2 3D 시각화** - Touch X 위치 및 방향 실시간 시각화
- ✅ **Docker 컨테이너 기반** - 환경 독립적 실행
- ✅ **완전 자동화** - 설치 스크립트로 원클릭 설치

## 🔬 테스트 환경

### 호스트 시스템

- **OS**: Ubuntu 20.04, 22.04
- **Docker**: 20.10 이상
- **X11**: X11 forwarding 지원

### 컨테이너 환경

- **Base Image**: `osrf/ros:foxy-desktop` (Ubuntu 20.04)
- **ROS 2**: Foxy Fitzroy
- **OpenHaptics SDK**: 3.4-0 Developer Edition
- **Touch X Driver**: 2022 버전

### 하드웨어

- **Geomagic Touch X**: USB 연결 (USB-A to USB-B)
- **USB Serial Port**: `/dev/ttyACM0` (자동 인식)

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

이 스크립트는 다음을 실행합니다:
- Touch X 드라이버 시작
- ROS 2 토픽 발행 시작
- **RViz2 GUI 자동 실행** - Touch X 위치 실시간 시각화

### RViz2에서 확인할 수 있는 것

- **TF 변환**: `touch_x_base` → `touch_x_ee` 축과 화살표
- **실시간 위치 업데이트**: Touch X를 움직이면 엔드 이펙터 위치가 실시간으로 업데이트
- **3D 공간 시각화**: Grid와 축으로 3D 공간에서 위치 확인
- **디버깅 정보**: Marker로 position, velocity, effort 값 표시 (옵션)

## 📊 제공되는 ROS 2 토픽

### 1. `/geomagic_touch_x/joint_states` (sensor_msgs/JointState)
- **position**: 6개 조인트 각도 (joint_angle_1~3, gimbal_angle_1~3)
- **velocity**: 조인트 각속도
- **effort**: 조인트 토크
- **발행 주기**: 200Hz

### 2. `/geomagic_touch_x/twist` (geometry_msgs/TwistStamped)
- **linear**: 엔드 이펙터 선속도 (x, y, z, 단위: m/s)
- **angular**: 엔드 이펙터 각속도 (x, y, z, 단위: rad/s)
- **발행 주기**: 200Hz

### 3. `/tf` (tf2_msgs/TFMessage)
- **변환**: `touch_x_base` → `touch_x_ee`
- **용도**: RViz2에서 시각화
- **발행 주기**: 200Hz

## 📝 사용 방법

### 데이터 모니터링

#### 빠른 실시간 모니터링 (권장)

```bash
# Position 값만 빠르게 보기
./monitor_touchx_fast.sh position

# Velocity만 빠르게 보기
./monitor_touchx_fast.sh velocity

# 전체 조인트 상태 (필터링됨)
./monitor_touchx_fast.sh joint_states
```

#### 상세 모니터링

```bash
./monitor_touchx.sh joint_states  # 전체 상세 출력
./monitor_touchx.sh twist         # 속도 정보
./monitor_touchx.sh tf            # TF 변환
./monitor_touchx.sh hz            # 토픽 주파수 확인
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

