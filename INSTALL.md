# 상세 설치 가이드

## Docker 이미지 빌드

### 기본 빌드

```bash
docker build -f Dockerfile.touchx_ros2 -t touchx_ros2:latest .
```

**예상 시간**: 10-20분 (인터넷 속도에 따라 다름)  
**이미지 크기**: 약 3-4GB

### 빌드 옵션

#### 캐시 없이 빌드 (문제 해결용)

```bash
docker build --no-cache -f Dockerfile.touchx_ros2 -t touchx_ros2:latest .
```

#### 빌드 과정 확인

```bash
docker build -f Dockerfile.touchx_ros2 -t touchx_ros2:latest . --progress=plain
```

## Dockerfile 내용

이 Dockerfile은 다음을 포함합니다:

1. **베이스 이미지**: `osrf/ros:foxy-desktop` (Ubuntu 20.04 + ROS 2 Foxy)
2. **시스템 패키지**: 빌드 도구, Qt5 라이브러리 등
3. **OpenHaptics SDK 3.4**: 자동 다운로드 및 설치
4. **Touch X 드라이버 2022**: 자동 다운로드 및 설치
5. **환경 변수 설정**: GTDD_HOME, LD_LIBRARY_PATH 등

## 빌드 과정

1. Ubuntu 20.04 + ROS 2 Foxy 기본 이미지 다운로드
2. 필수 시스템 패키지 설치
3. OpenHaptics SDK 다운로드 및 설치 (~100MB)
4. Touch X 드라이버 다운로드 및 설치 (~10MB)
5. 환경 변수 및 설정 디렉토리 생성
6. ROS 사용자 생성 및 권한 설정

## 빌드 후 확인

```bash
# 이미지 확인
docker images | grep touchx_ros2

# 이미지 상세 정보
docker inspect touchx_ros2:latest

# 컨테이너 실행 테스트
docker run --rm touchx_ros2:latest echo "Build successful"
```

## 이미지 저장/로드

### 이미지 저장 (백업)

```bash
docker save -o touchx_ros2.tar touchx_ros2:latest
# 파일 크기: 약 3-4GB
```

### 이미지 로드 (다른 컴퓨터에서)

```bash
docker load -i touchx_ros2.tar
```

## 문제 해결

### 네트워크 오류

빌드 중 네트워크 오류가 발생하면:
- 인터넷 연결 확인
- 방화벽 설정 확인
- 프록시 설정 확인

### 디스크 공간 부족

```bash
# 디스크 공간 확인
df -h

# 기존 Docker 이미지 정리
docker system prune -a
```

### 빌드 시간이 너무 오래 걸림

빌드는 최초 1회만 필요합니다. 이후에는:
- 컨테이너만 재시작하면 됩니다
- 이미지를 저장해두면 다른 컴퓨터에서 바로 사용 가능합니다

