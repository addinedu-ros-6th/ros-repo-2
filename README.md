# ros-repo-2
파이널 프로젝트 2조 저장소. 푸드코트 서빙 로봇
## 1. 개요
![main_image](https://github.com/user-attachments/assets/16a02216-99ca-4f24-ab51-0b98795a14fa)
### 1.1. 프로젝트 설명
서비?
서비는 주문 관리, 서빙, 그릇 회수, 매장 상태 모니터링 등 매장 운영 전반을 체계적인 관리하는 시스템
### 1.2. 기술 스택
|분류|기술|
|-----|-----|
|개발 환경|Linux|
|개발 언어|Python|
|하드웨어|RaspberryPi, Arduino|
|딥러닝 프레임워크|Ultralytics YOLOv8|
|통신 프로토콜|UDP, ROS|
|DBMS|MySQL|
|UI|PyQT|
|영상 처리|openCV|
|자율 주행|ROS, ArUco Marker, Lidar|
### 1.3. 팀 구성
|성명|담당|
|----|-----|
|김재창 <br> (Project Leader)| 로봇 위치 보정<br> 통신 서버 구축<br>주문 및 로봇 작업 할당 관리| 
|김주연| 충돌 회피 알고리즘 개발<br>관리자용 GUI 구현<br로봇 작업 할당 알고리즘 개발|
|윤희천| 자세 추정 모델 개발<br>테이블 상태 체크 모델 개발<br>데모 환경 설계| 
|서영환| GUI 전반 설계 및 구현<br>DB 설계 및 구현<br>딥러닝 모델 입출력 모델 통합|
|조전욱| Behavior Tree 구현 <br>로봇 경로계획 및 주행<br>시스템 통합 및 디버깅|
