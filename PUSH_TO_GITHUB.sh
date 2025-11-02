#!/bin/bash
# GitHub에 Push하는 스크립트

echo "========================================="
echo "GitHub 리포지토리에 Push"
echo "========================================="
echo ""

# GitHub 리포지토리 URL 확인
if ! git remote | grep -q origin; then
    echo "GitHub 리포지토리 URL을 입력하세요:"
    echo "예: https://github.com/username/repo-name.git"
    read -p "URL: " REPO_URL
    
    if [ -z "$REPO_URL" ]; then
        echo "❌ URL이 입력되지 않았습니다."
        exit 1
    fi
    
    git remote add origin "$REPO_URL"
    echo "✅ 원격 리포지토리 추가 완료"
fi

# 브랜치 이름을 main으로 변경
git branch -M main 2>/dev/null || true

echo ""
echo "GitHub에 Push 중..."
git push -u origin main

echo ""
echo "✅ Push 완료!"

