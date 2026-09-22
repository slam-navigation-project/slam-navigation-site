# SLAM 기반 다중 AMR 자율주행 시스템

- **과목:** 융합캡스톤디자인 2 (C057-5) 1조
- **팀원:** 박주원, 윤성빈
- **배포 사이트:** [https://slam-navigation-project.github.io/slam-navigation-site/](https://slam-navigation-project.github.io/slam-navigation-site/)

---

## 📝 개발 로그 (`_posts`) 작성 가이드

새로운 개발 일지나 실험 결과를 올릴 때 별도의 홈 화면 코드 수정 없이 `_posts/` 폴더에 마크다운(`.md`) 파일만 추가하면 Jekyll 엔진이 자동으로 최신 글을 메인에 게시합니다.

### 1. 파일 이름 규칙 (필수)
파일 이름은 반드시 **`YYYY-MM-DD-제목.md`** 형식이어야 합니다. 날짜가 빠지면 사이트에 노출되지 않습니다.

- 올바른 예시: `_posts/2026-09-23-lidar-driver-test.md`
- 잘못된 예시: `_posts/test.md`, `_posts/2026_09_23_test.md`

### 2. 머리말 (Front-matter) 템플릿
마크다운 파일의 가장 첫머리에 아래 서식을 반드시 포함해야 합니다:

```markdown
---
layout: post
title: "게시글 제목 (예: 2D LiDAR 센서 연동 및 SLAM Toolbox 맵핑 테스트)"
date: 2026-09-23 15:00:00 +0900
categories: [SLAM, Test]
tags: [ROS2, LiDAR, RaspberryPi]
---

여기에 본문 내용을 마크다운으로 자유롭게 작성합니다.
첫 문단은 사이트 홈 화면의 글 카드 요약(Excerpt)으로 자동 출력됩니다.
