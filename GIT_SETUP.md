# GitHub 리포지토리 설정 가이드

## 1. GitHub에서 새 리포지토리 생성

1. GitHub에 로그인
2. "New repository" 클릭
3. Repository name: `geomagic-touch-x-ros2` (또는 원하는 이름)
4. Public 또는 Private 선택
5. **"Initialize this repository with a README" 체크하지 않기**
6. "Create repository" 클릭

## 2. 로컬 리포지토리를 GitHub에 연결

```bash
cd /home/medisc/Desktop/TouchX-Repo

# GitHub 리포지토리 URL 추가 (아래 <your-username>과 <repo-name> 수정)
git remote add origin https://github.com/<your-username>/<repo-name>.git

# 기본 브랜치를 main으로 설정
git branch -M main

# GitHub에 push
git push -u origin main
```

## 예시

```bash
cd /home/medisc/Desktop/TouchX-Repo
git remote add origin https://github.com/myusername/geomagic-touch-x-ros2.git
git branch -M main
git push -u origin main
```

## 추가 변경사항 push

나중에 파일을 수정한 후:

```bash
cd /home/medisc/Desktop/TouchX-Repo
git add .
git commit -m "Update: 설명"
git push
```

