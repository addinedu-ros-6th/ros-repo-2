# ros-repo-2
파이널 프로젝트 2조 저장소. 푸드코트 서빙 로봇
## 시연 풀영상
<!-- - 아래의 이미지를 클릭하면 영상을 볼 수 있습니다.
[![영상보기](https://github.com/user-attachments/assets/f78e1bf7-a68e-4b3e-a502-9ea3d6d49f37)](https://drive.google.com/file/d/11n8TmEqF2lIWvv2bNaJOQPRYmwpDtWcu/view) -->
![시연영상](https://github.com/user-attachments/assets/a36f735f-1707-4b5a-ad60-d58fd0a674d8)


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
| 로봇 회수 제어 | 고객 호출 시 해당 테이블로 주행, 회수 차례 순서 출력, 대기시간 기준치 초과 시 퇴식구로 주행 | 
| 주행 상태 제어 | 로봇 주행 - 경로 요청, 이동, 이동 완료 여부 체크, 주차 |
### 2.2. 시스템 구성도
![system_structure](https://github.com/user-attachments/assets/df17b161-81dc-4d0f-9514-bc72f5c13e40)
### 2.3. 주요 시나리오
![serving_synario](https://github.com/user-attachments/assets/bc9a5ac8-7612-4432-b5de-f85510f6f0c9)
<br>![retrieve](https://github.com/user-attachments/assets/b0137171-ab0b-4f39-9397-3bc815f83644)
### 2.4. 통신 프로토콜
![network_protocol](https://github.com/user-attachments/assets/a82f8b8d-4349-42eb-a1ee-5a02a0956240)
### 2.4. GUI 설계
Customer GUI<br>
![38_customer_gui](https://github.com/user-attachments/assets/bf2d7955-6865-4cff-9ddc-127d9598bacd)
<br><br>Vendor GUI<br>
![38_vendor_gui](https://github.com/user-attachments/assets/04c8237e-2256-4aa8-9dcf-87791059366b)
<br><br>Manager GUI<br>
![47_manager_gui](https://github.com/user-attachments/assets/94979fd5-04fb-4eea-bfb1-b5b87eb980c6)
### 2.4. 데이터베이스 설계
![DB_structure](https://github.com/user-attachments/assets/dc8d1728-9747-4861-ad4e-f8d0853a5b8b)

### 2.5. 로봇 상태 구현(Behavior Tree)
![bt](https://github.com/user-attachments/assets/7e1fc01a-2304-48f0-af42-0f095d709cea)
<br> Behavior Tree를 활용하여 로봇의 상태를 모듈화하여 관리
<br> 1) 배터리 상태 관리
<br>
<br> 2) Create Path
<br>   - 대기 또는 충전소로 복귀 중인 상태일 때만 임무 받기 가능
<br>   - 목표 좌표를 TaskManager에게 전달받으면 PathPlanner에게 경로 요청
<br>
<br> 3) Movement
<br>   - Waypoint를 체크하며 이동이 완료되었는지 체크
<br>   - 벽과의 거리를 체크해서 벽 회피 이동을 해야하는지 체크
<br>   - 목표 좌표와의 방향 백터를 활용해서 오차 각도를 체크
<br>   - PID 제어를 통해 오차 각도를 조절
<br>   - Odometry로 이동 거리를 체크
<br>   - Amcl로 허용 범위 내에 이동이 완료했는지 체크
<br>
<br> 4) Parking
<br>   - 목적지에 도달하면 주차를 시도
<br>   - Search: 로봇이 회전하며 Aruco Marker를 찾음
<br>   - Aligning: 로봇이 Aruco Marker를 정면에서 바라보도록 만드는 상태
<br>   - Yawing: 라이다로 직선을 검출해서 테이블을 정면에서 바라보도록 만드는 상태 
<br>   - Approaching: 테이블로 가깝게 다가가는 상태
<br>
<br> 5) Robot Standby
<br>   - 목적지에 도달하면 AI로 부터 서빙 및 회수가 완료 신호가 올때까지 대기
<br>

## 3. 기능 구현
### 3.1. 로봇 간 충돌 회피 알고리즘
### 3.2. 서빙 주행
서빙 (1/4)
![2조-발표 자료(최종)](https://github.com/user-attachments/assets/af096b01-f8fd-4556-bf95-a0c337f10c9e)
<br>서빙 (2/4)
<br>서빙(정상주행)
<br>충전소 > 매장 > 테이블 이동
<!-- <br> -아래의 이미지를 클릭하면 영상을 볼 수 있습니다. -->
![40_serving_full_video_compressed_2](https://github.com/user-attachments/assets/9e872ffa-2b13-4092-9902-6e1346a03066)
<br>
<br>
서빙(3/4)
<br>서빙(충돌 회피 주행)
<br>넓은 공간: 오른쪽으로 서로를 회피
<!-- <br> - 아래의 이미지를 클릭하면 영상을 볼 수 있습니다. 
<br> -->
![41_avoidance_robot_1_compressed](https://github.com/user-attachments/assets/9c513595-ccf0-45ff-83d6-782a0b6b85d8)
<br>좁은 공간: 한대만 오른쪽으로 회피
<!-- <br> - 아래의 이미지를 클릭하면 영상을 볼 수 있습니다. 
<br> -->
![41_avoidance_robot_2_compressed](https://github.com/user-attachments/assets/71e77e11-cf44-4f68-8cc3-7915ccd40682)
<br>
<br>
서빙(4/4)
<br> 서빙(충돌 회피 주행)
<br> 사람이 뛰는 동시에 로봇 방향으로 이동하는 경우 주행 정지
<!-- <br> - 아래의 이미지를 클릭하면 영상을 볼 수 있습니다.
[![영상보기](https://github.com/user-attachments/assets/fba3cc81-83bc-4fbc-bce1-7869b4036cab)](https://drive.google.com/file/d/1hi7HLdslX-P8V-ys0-Rwj_h3-9rZysuG/view) -->
![42_avoidance_person_resized_cropped](https://github.com/user-attachments/assets/922a309c-9e93-452e-b6af-2a7af36cb758)
<br>
<br>
### 3.3. 회수 주행
#### 3.3.1. 회수 요청
<br> 고객이 빈 접시 회수를 요청
<br> TaskManager가 로봇에게 임무를 할당
#### 3.3.2. 로봇 임무 수행
<br> 1) 테이블로 이동
<br> 2) AI체크 테이블 위의 빈 접시가 모두 회수 되었는지 확인
<br> 3) 테이블 위의 접시가 모두 회수되면 퇴식구로 이동
<br> 4) 잔여 작업이 존재하면 다음 작업을 수행하고 없으면 충전소로 이동
<!-- <br> - 아래의 이미지를 클릭하면 영상을 볼 수 있습니다. -->
![44_retrieve_full_video_resized](https://github.com/user-attachments/assets/12ec8451-ec19-4e55-9875-60e617391a78)
<br>

## 4. 핵심 기술
### 4.1 위치 보정
아르코 마커와 라이다
<br>주행 결과: 목적지까지 가로 거리 기준 5cm 이내로 도달 가능
<br>![주차](https://github.com/user-attachments/assets/752e2d25-b052-4e81-b9df-2ab72ba785af)

### 4.2 딥러닝

#### **테이블 상태 체크 모델**
![table_check_demo](https://github.com/user-attachments/assets/3b23542d-b166-44f7-9368-1b7fb61b6aa9)

**목표**  
- 로봇이 테이블 위의 상태(Empty/Not Empty)를 실시간으로 인식하여 음식 회수 작업을 자동화하고 효율성을 향상시키는 것.

**사용 기술**  
- **모델: MobileNetV2**  
  - 경량화된 CNN 구조로 실시간 예측 및 제한된 리소스 환경에서도 높은 성능 제공.
  - ImageNet 사전 학습 가중치로 초기화하여 적은 데이터로도 학습 속도와 성능을 확보.
- **프레임워크**: TensorFlow, Keras

**데이터 처리**  
- 2,000개 이상의 테이블 이미지를 수집하여 두 가지 상태로 분류(Empty/Not Empty).
- **데이터 증강**: 회전, 이동, 밝기 조정 등으로 모델의 일반화 성능 향상.
- 모든 이미지 크기를 224x224로 조정 및 정규화(rescale = 1.0/255).
![image](https://github.com/user-attachments/assets/0cfc6ce2-579e-4ecf-9004-5367c056d9c7)
![image](https://github.com/user-attachments/assets/9a994db4-be82-4878-afdc-6c8206540131)
![image](https://github.com/user-attachments/assets/4ab8f1d1-9d46-45f3-a9f7-018019d23cb1)



**성과**  
- 검증 정확도 **95.25%** 달성.
![image](https://github.com/user-attachments/assets/47b21211-caad-4615-80bd-e836e9c70bed)

- 안정적인 손실 감소를 통해 과적합 방지 및 높은 일반화 성능 확보.
- **후처리**를 활용하여 테이블 번호와 상태를 매핑, 로봇 동선을 최적화.
![image](https://github.com/user-attachments/assets/beadf8ec-2d0b-4940-9d4f-180adc8d8a51)

---

#### **자세 추정 모델**
![running_forward_demo](https://github.com/user-attachments/assets/59bf3042-6eac-4449-97fb-82e95f296783)
![sitting_demo](https://github.com/user-attachments/assets/2ff34ce3-5208-41d4-bd69-3f856bbca297)
![standing_demo](https://github.com/user-attachments/assets/cb8e0062-84ea-4730-ad2d-5676080964b5)
![walk_side_demo](https://github.com/user-attachments/assets/f10da982-6d7f-4ecb-bd87-decfd8ea8434)


**목표**  
- 로봇이 고객의 행동(걷기, 뛰기, 서기, 앉기)과 방향(정면/다른 방향)을 실시간으로 인식하여 안전한 매장 운영을 지원.

**사용 기술**  
- **모델: YOLOv8 Pose nano & Bidirectional LSTM**  
  - **YOLOv8 Pose nano**: 17개 관절 좌표를 실시간으로 추출해 행동 상태 분석.
  - **Bidirectional LSTM**: 관절 좌표의 시계열 데이터를 학습하여 행동 패턴 분류.
- **프레임워크**: TensorFlow, Keras

**데이터 처리**  
- 15 프레임 단위로 관절 좌표를 시퀀스화하여 학습 데이터 구성.
- 다양한 행동과 방향 데이터를 수집하여 모델의 다양성 확보.
![image](https://github.com/user-attachments/assets/ca2b14ca-81f7-4470-bb77-ae24f7a909f1)
![image](https://github.com/user-attachments/assets/2e1c4204-0e58-4d7b-8743-e6930a4e9fe0)



**성과**  
- 검증 정확도 **95.17%** 달성.
![image](https://github.com/user-attachments/assets/b062bfd0-ffa8-452e-b2f2-5db24aad96f9)

- 충돌 회피 알고리즘에 기여하여 로봇이 안전한 경로로 이동 가능.
- 이동 방향 정보(정면/다른 방향) 활용으로 로봇 동선을 최적화.
![image](https://github.com/user-attachments/assets/af7792bd-16a8-49d4-af11-571cd0b60499)

---

#### **결론**

- **테이블 상태 체크 모델**은 음식 회수 작업의 정확도와 효율성을 높이고, **행동 추정 모델**은 고객의 행동과 방향을 실시간으로 파악해 로봇의 안전성과 서비스 품질을 향상.  
- 두 모델 모두 데이터 증강, 정규화 등 **과적합 방지 기법**을 적극 활용하여 높은 검증 정확도로 실시간 애플리케이션에 적합한 결과를 도출.


## 5. 트러블 슈팅
### 5.1 기존 패키지 문제 - 노이즈와 확률
기존의 Nav2 패키지는 확률 기반 추정으로 인해 오차가 발생
<br> 좁은 필드에서 생기는 문제
<br> 1) 벽과 충돌: Costmap의 inflation Layer를 너무 줄일 경우 벽과 충돌하는 문제 발생
<br> 2) 경로 변경 반복: Costmap 계산을 새로 할 때마다 좁은 통로를 갈지 말지 결정이 지연
![4 2 1  - 확률 문제](https://github.com/user-attachments/assets/715d7348-abbf-447f-b5af-7dd270dbbcae)

### 5.2 해결 방안
경로 계획
<br>A* 알고리즘 사용
<br>Slam으로 제작된 맵 파일로 경로 계획(정적데이터)
<br>경로가 동적으로 변경되는 문제 해결
<br>![1](https://github.com/user-attachments/assets/3d0074d0-3194-495e-87e0-45a34a6b5ca0)

로봇 주행 구현
<br> 2-1) 이동한 거리 계산
<br> - 미끄러짐 등의 드리프트로 인한 오차가 누적되지만 단거리에서는 꽤 정확한 Odomeytry
<br> - Odomeytry로 이동한 거리로 목적지까지의 남은 거리를 계산

<br> 2-2) 목적지 도착 여부 확인
<br> - 확률로 위치를 추정하는 AMCL를 매번 좌표가 흔들린다는 문제가 발생.
<br> - 오차범위 내에서는 꽤 정확함.
<br> - 로봇이 waypoint에 도착하면 로봇의 AMCL 좌표가 허용 범위 안인지 확인
<br> - AMCL좌표는 확률 기반이라 때론 로봇이 벽과 충돌하는 문제가 발생
<br> - 라이다로 벽과의 거리를 체크해서 벽과 멀어지는 알고리즘으로 해결
<br>
![개선후](https://github.com/user-attachments/assets/b04e9871-afd6-43d0-a177-cab2dd25f2fa)


## 6. 개발 환경
개발 기간 : 24.10.17 ~ 24.11.25 (5주
![timeline](https://github.com/user-attachments/assets/ae7ca6c8-e6dd-41f2-94b1-6f3422da15b0)
<br><br><br>
개발 방법론 : V모델<br>
![vmodel](https://github.com/user-attachments/assets/a6e1c5d1-9abc-46c3-a782-83bd78243baf)
