# SLAM Autonomous Driving Jekyll Theme & Site Template

GitHub Pages로 바로 호스팅 가능한 **SLAM 기반 목적지 자율주행 프로젝트 전용 Jekyll 테마/사이트 템플릿**입니다.

## 🚀 빠른 시작 가이드

### 방법 1: GitHub Pages에 바로 올리기
1. 새 GitHub 저장소(`my-slam-site` 또는 `[username].github.io`)를 생성합니다.
2. 본 압축 파일의 모든 파일과 폴더를 해당 저장소의 루트 디렉토리에 푸시합니다.
3. GitHub 저장소의 **Settings > Pages** 메뉴로 이동합니다.
4. **Source**를 `Deploy from a branch` (Branch: `main` / `root`)로 설정하고 저장합니다.
5. `_config.yml`의 `url`, `baseurl`, `author.github`을 사용자 계정 정보로 수정합니다.

### 방법 2: 로컬에서 테스트 실행
```bash
# Ruby 및 Bundler가 설치된 환경에서
bundle install
bundle exec jekyll serve
# 브라우저에서 http://localhost:4000 접속
```

## 📁 디렉토리 구조
```
├── _config.yml         # 사이트 전역 메타데이터 및 빌드 설정
├── Gemfile             # Jekyll 의존성 젬 정의
├── _includes/          # 헤더, 푸터, 메타태그 등 공통 템플릿 컴포넌트
├── _layouts/           # 페이지/포스트/홈 기본 레이아웃 정의
├── _posts/             # 마크다운 포스트 및 개발 일지 저장 폴더
├── assets/
│   ├── css/style.css   # 다크 테마 기반 반응형 스타일시트
│   ├── js/main.js      # 프론트엔드 스크립트
│   └── images/         # 스크린샷 및 다이어그램 저장소
├── architecture.md     # 시스템 아키텍처 상세 페이지
├── about.md            # 프로젝트 소개 페이지
└── index.html          # 메인 랜딩 페이지
```
