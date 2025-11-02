# Geomagic Touch X + ROS 2 Foxy

USB-connected Geomagic Touch X haptic device를 ROS 2 Foxy와 연동하는 솔루션입니다.

## 🚀 빠른 시작

### 1. 자동 설치

```bash
git clone https://github.com/RACCOOONkim/touch-x-ros2.git
cd touch-x-ros2
chmod +x SETUP.sh
./SETUP.sh
```

### 2. Touch X 설정 (최초 1회)

```bash
docker exec -it touchx_ros2 bash -c "
export GTDD_HOME=/usr/share/3DSystems
export LD_LIBRARY_PATH=/opt/OpenHaptics/Developer/3.4-0/lib64:\$LD_LIBRARY_PATH
export DISPLAY=:0
sudo chmod 666 /dev/ttyACM0
Touch_Setup
"
```

### 3. 실행

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

### USB 디바이스 인식 안 됨
```bash
sudo chmod 666 /dev/ttyACM0
```

### RViz2 GUI가 안 열림
```bash
xhost +local:docker
```

## 📚 라이센스

MIT License

