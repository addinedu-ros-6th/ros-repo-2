# ros-repo-2
파이널 프로젝트 2조 저장소. 푸드코트 서빙 로봇
## 발표 자료
[발표자료_링크](https://docs.google.com/presentation/d/1bO1ZOO_CHZOZPhwl_kTbt7kyJYsiErI16zEfxeV0qEo/edit?usp=sharing)
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
## 2. 시스템 설계
### 2.1. 주요 기능 리스트
| 기능    | 기능 설명 |
|----------|---------------|
| 고객 서비스 및 <br>운영, 매장 관리자 | 고객 주문 기능 및 주문 현황, 매장 현황, 로봇 현황 조회 |, 
| 로봇 서빙 제어 | 서빙 로봇 자동 호출, 서빙 로봇 자동 배정, 서빙 로봇 자동 배정 (그룹)|
| 로봇 회수 제어 | 고객 호출 시 해당 테이블로 주행, 회수 차례 순서 출력, 대기시간 기준치 초과 시 퇴식구로 주행 - 어디까지 구현되었는지 확실하지 앟음 | 
| 주행 상태 제어 | 로봇 주행 - 여기도 마찬가지 
### 2.2. 시스템 구성도
![system_structure](https://github.com/user-attachments/assets/df17b161-81dc-4d0f-9514-bc72f5c13e40)
### 2.3. 주요 시나리오
![serving_synario](https://github.com/user-attachments/assets/bc9a5ac8-7612-4432-b5de-f85510f6f0c9)
<br>![retrieve](https://github.com/user-attachments/assets/b0137171-ab0b-4f39-9397-3bc815f83644)
### 2.4. 통신 프로토콜
![network_protocol](https://github.com/user-attachments/assets/a82f8b8d-4349-42eb-a1ee-5a02a0956240)
### 2.4. GUI 설계
Customer GUI<br>
![customer_gui](https://github.com/user-attachments/assets/30148a26-b684-46f7-859a-590b0379e447)
<br><br>Vendor GUI<br>
![vendor_gui](https://github.com/user-attachments/assets/d350bd1c-1639-4718-b318-4f54f543311c)
<br><br>Manager GUI<br>
![manager_gui1](https://github.com/user-attachments/assets/17901211-5531-4e51-b82f-1519852eff81)
### 2.4. 데이터베이스 설계
![DB_structure](https://github.com/user-attachments/assets/dc8d1728-9747-4861-ad4e-f8d0853a5b8b)
## 3. 기능 구현
### 3.1. 로봇 간 충돌 회피 알고리즘
### 3.2. 서빙 주행
서빙 (1/4)
![2조-발표 자료(최종)](https://github.com/user-attachments/assets/af096b01-f8fd-4556-bf95-a0c337f10c9e)
<br>서빙 (2/4)
<br>서빙(정상주행)
<br>충전소 > 매장 > 테이블 이동
<br> -아래의 이미지를 클릭하면 영상을 볼 수 있습니다.
[![영상보기](https://github.com/user-attachments/assets/f78e1bf7-a68e-4b3e-a502-9ea3d6d49f37)](https://drive.google.com/file/d/1JY-tyPiFlq8X7RqE-y3h2twX7MDIrPx1/view)
<br>
<br>
서빙(3/4)
<br>서빙(충돌 회피 주행)
<br>넓은 공간: 오른쪽으로 서로를 회피
<br>좁은 공간: 한대만 오른쪽으로 회피
<br> - 아래의 이미지를 클릭하면 영상을 볼 수 있습니다.
[![영상보기](https://github.com/user-attachments/assets/7a679b34-68f3-4323-98d8-dfaf8cad23dc)](https://drive.google.com/file/d/1VPksvb4nGBH0s23Fag2mS7aWTKAE_U5n/view)
[![영상보기](https://github.com/user-attachments/assets/576291cc-bc7a-41db-9745-1b99d7f73df8)](https://drive.google.com/file/d/1eEzcYA_rjczE5vspLd7MERSeqw1W9p40/view)
<br>
<br>
서빙(4/4)
<br> 서빙(충돌 회피 주행)
<br> 사람이 뛰는 동시에 로봇 방향으로 이동하는 경우 주행 정지

### 3.3. 회수 주행

## 4. 핵심 기술
### 4.1 위치 보정
아르코 마커와 라이다
### 4.2 딥러닝
자세추정 모델
<br> 테이블 상태 체크 모델
## 5. 개발 환경
개발 기간 : 24.10.17 ~ 24.11.25 (5주
![timeline](https://github.com/user-attachments/assets/ae7ca6c8-e6dd-41f2-94b1-6f3422da15b0)
<br><br><br>
개발 방법론 : V모델<br>
![vmodel](https://github.com/user-attachments/assets/a6e1c5d1-9abc-46c3-a782-83bd78243baf)
