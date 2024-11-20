"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math
import numpy as np

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.ndimage import distance_transform_edt

class AStarPlanner:
    """
    A* (A-star) 알고리즘을 사용한 경로 탐색을 담당하는 플래너 클래스.

    이 클래스는 A* 알고리즘을 이용해 주어진 지도에서 시작 지점부터 목표 지점까지의 최적 경로를 탐색합니다.
    지도는 그리드 형식으로 표현되며, 장애물 맵, 이동 가능한 경로 등을 설정할 수 있습니다. 로봇의 크기와 
    장애물 간의 패딩 처리, 맵의 해상도 등 다양한 설정을 지원합니다.

    속성:
    - resolution (float): 그리드 해상도 [m/칸], 한 칸의 실제 길이를 나타냄.
    - rr (float): 로봇 반경 [m], 로봇의 크기를 고려하여 경로 계산 시 장애물과의 거리 확보에 사용.
    - padding (int): 장애물 패딩 크기, 장애물 주변의 안전 거리를 확보하기 위한 패딩 값.
    - min_x, min_y (int): 맵의 최소 x, y 좌표.
    - max_x, max_y (int): 맵의 최대 x, y 좌표.
    - obstacle_map (ndarray): 장애물 맵, 그리드 상의 장애물 위치를 저장.
    - x_width, y_width (int): 맵의 너비와 높이 (그리드의 개수).
    - motion (list): 가능한 이동 방향 및 각 방향에 따른 이동 비용을 나타냄.
    - map_resolution (float): 맵의 해상도 [m/픽셀], 맵 파일에서 픽셀 크기에 따른 실제 거리 변환에 사용.
    - map_origin (tuple): 맵의 원점 좌표 (맵의 좌표계에서 기준점이 되는 위치).
    
    주요 메서드:
    - load_map(): 맵 데이터를 불러오고, 장애물의 좌표를 설정하는 함수.
    - calc_obstacle_map(ox, oy): 장애물 좌표를 기반으로 장애물 맵을 생성하는 함수.
    - get_motion_model(): 이동 가능한 방향(상하좌우, 대각선 등)과 각 이동의 비용을 반환하는 함수.
    - planning(sx_real, sy_real, gx_real, gy_real): A* 알고리즘을 통해 시작 지점(sx, sy)에서 목표 지점(gx, gy)까지의 경로를 탐색하는 함수.
    - calc_final_path(goal_node, closed_set): 목표 노드에서 시작 노드까지의 최종 경로를 생성하는 함수.
    """

    def __init__(self, resolution, rr, padding):


        self.resolution = resolution
        self.rr = rr
        self.padding = padding
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model() # 이동할 수 있는 방향 설정. ( 4방향으로 설정되어 있다.)

        self.resolution = 1
        self.map_resolution = 1
        self.map_origin = (0, 0)
        ox, oy = self.load_map() # 장애물 좌표를 가져온다.

        self.calc_obstacle_map(ox, oy)

        print("Map loading done!")

    def load_map(self):
        """
        맵 파일을 로드하고, 장애물 좌표를 설정하는 함수.

        이 함수는 YAML 파일과 PGM 파일에서 맵 데이터를 읽어와, 맵의 해상도와 원점을 설정하고,
        맵의 픽셀 데이터를 기반으로 장애물 위치를 결정합니다. 장애물 주변에 안전 거리를 확보하기 위해
        패딩 처리를 수행하여, 로봇이 장애물에 가까이 가지 않도록 합니다.

        반환값:
        - ox (list): 장애물의 x 좌표 리스트.
        - oy (list): 장애물의 y 좌표 리스트.
        """
        print("Loading map start!")

        # 맵 정보를 담고 있는 YAML 파일의 경로를 설정하고 데이터를 로드
        map_yaml_file = os.path.join(get_package_share_directory('minibot_navigation2'), 'maps', 'map_final_2.yaml')        
        map_yaml_data = yaml.full_load(open(map_yaml_file))

        # 맵 해상도(m/픽셀)와 맵의 원점 좌표 설정
        self.map_resolution = map_yaml_data['resolution']    # 해상도(m/픽셀)
        self.map_origin = map_yaml_data['origin']    # 맵의 원점 좌표 [x, y]

        # PGM 이미지 파일 경로를 설정
        map_pgm_file = os.path.join(get_package_share_directory('minibot_navigation2'), 'maps', map_yaml_data['image'])

        # PGM 파일을 읽어들여 맵 데이터를 로드
        with open(map_pgm_file, 'rb') as pgmf:
            # 파일의 모든 줄을 읽어와 주석 제거
            pgm_data = [line for line in pgmf.readlines() if not line.startswith(b'#')]
            
            # PGM 파일의 크기(너비와 높이)를 설정
            map_width, map_height = map(int, pgm_data[1].split())
            
            # PGM 파일의 픽셀 데이터를 배열로 변환하고 맵 데이터로 사용
            map_data = np.array(list(map(int, pgm_data[3])))
            
            # 픽셀 값이 210 이하인 부분을 장애물(0)로, 그 이상의 값은 이동 가능 구역(100)으로 설정
            map_data[map_data <= 210] = 0
            map_data[map_data > 210] = 100
            
            # 맵 데이터를 맵의 높이와 너비에 맞게 재구성
            map_data = map_data.reshape((map_height, map_width))
            
            # 맵이 상하 반전된 상태로 불러와져 있기 때문에 이를 수정
            map_data = np.flip(map_data, axis=0)

        # 장애물 좌표를 저장할 리스트 초기화
        ox, oy = [], []
        
        # 패딩이 포함된 맵 데이터를 복사하여 저장
        padded_map_data = map_data.copy()
        
        # 맵의 각 좌표에 대해 장애물을 탐색
        for i in range(map_height):
            for j in range(map_width):
                # 장애물이 있는 좌표(0)일 경우 해당 좌표를 장애물 리스트에 추가
                if map_data[i][j] == 0:
                    ox.append(j)
                    oy.append(i)

                    # 장애물 주변에 패딩을 적용하여 로봇이 가까이 가지 않도록 함
                    for dx in range(-self.padding, self.padding + 1):
                        for dy in range(-self.padding, self.padding + 1):
                            # 패딩 영역이 맵의 범위를 벗어나지 않도록 확인한 후 패딩 적용
                            if 0 <= j + dx < map_width and 0 <= i + dy < map_height and padded_map_data[i + dy][j + dx] != 0:
                                padded_map_data[i + dy][j + dx] = 0
                                ox.append(j + dx)
                                oy.append(i + dy)

        # 장애물의 x, y 좌표 리스트 반환
        return ox, oy


    class Node:
        def __init__(self, x, y, cost, parent_index, vector = None):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index
            self.vector = vector

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx_real, sy_real, gx_real, gy_real):
        """
        A* 알고리즘을 사용하여 시작 지점에서 목표 지점까지의 최적 경로를 탐색하는 함수.

        매개변수:
        - sx_real (float): 시작 지점의 실제 x 좌표 [m].
        - sy_real (float): 시작 지점의 실제 y 좌표 [m].
        - gx_real (float): 목표 지점의 실제 x 좌표 [m].
        - gy_real (float): 목표 지점의 실제 y 좌표 [m].

        반환값:
        - rx (list): 최종 경로의 x 좌표 리스트 (그리드 상의 좌표).
        - ry (list): 최종 경로의 y 좌표 리스트 (그리드 상의 좌표).
        - tpx (list): 경로에서 방향이 바뀐 지점의 x 좌표 리스트 (실제 맵 좌표).
        - tpy (list): 경로에서 방향이 바뀐 지점의 y 좌표 리스트 (실제 맵 좌표).
        - tvec_x (list): 각 방향이 바뀐 지점에서의 x 방향 벡터 리스트.
        - tvec_y (list): 각 방향이 바뀐 지점에서의 y 방향 벡터 리스트.

        주요 기능:
        - 실제 좌표를 그리드 인덱스로 변환하여 A* 탐색에 적합한 형태로 변환.
        - A* 알고리즘을 통해 시작 노드에서 목표 노드까지의 최적 경로를 탐색.
        - 경로 탐색 과정에서 방향이 바뀌는 지점을 기록하여 반환.
        - 탐색이 완료된 후, 최종 경로를 실제 좌표로 변환하여 반환.

        A* 알고리즘의 주요 탐색 단계:
        1. 시작점과 목표점을 그리드 인덱스로 변환.
        2. 오픈 셋과 클로즈드 셋을 사용해 경로를 탐색.
        3. 각 노드의 비용과 맨해튼 거리를 이용해 최적 경로를 찾아가는 과정.
        4. 경로가 완료되면, 방향이 바뀐 지점을 계산하고, 경로를 최종적으로 반환.
        """
        sx = (sx_real - self.map_origin[0]) / self.map_resolution
        sy = (sy_real - self.map_origin[1]) / self.map_resolution
        gx = (gx_real - self.map_origin[0]) / self.map_resolution
        gy = (gy_real - self.map_origin[1]) / self.map_resolution
        
        print(sx_real, sy_real, gx_real, gy_real)
        print(sx, sy, gx, gy)

        
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        is_starting = True     # navigation으로 스타팅 포인트가 padding 벗어났을 때 verifying 안하고 넘어가는 용도
        
        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                # key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
                key=lambda o: open_set[o].cost + self.calc_manhattan(goal_node, open_set[o]))
            
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                goal_node.vector = current.vector
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # 가능한 모든 이동 방향(motion)을 탐색
            for i, _ in enumerate(self.motion):

                is_turned = 0                                       # 방향이 변경되었는지 여부를 확인하기 위한 플래그 초기화
                before_vector = (0, 0)                              # 이전 노드에서 현재 노드로의 이동 벡터를 계산하기 위한 변수 초기화
                now_vector = (self.motion[i][0], self.motion[i][1]) # 현재 탐색 중인 방향에 따른 이동 벡터 설정

                # 현재 노드의 부모 노드가 closed_set에 있으면, 이전 노드와의 벡터를 계산
                if closed_set.get(current.parent_index):
                    before_node = closed_set[current.parent_index]

                    # 이전 노드에서 현재 노드로의 이동 벡터를 계산
                    before_vector = (current.x - before_node.x, current.y - before_node.y)

                    # 이전 벡터와 현재 벡터가 다르면, 방향이 바뀐 것으로 간주
                    is_turned = before_vector != now_vector


                # 새로운 이웃 노드를 생성하고, 이동 비용을 계산
                # 방향이 바뀐 경우, 추가 비용을 부과
                node = self.Node(current.x + self.motion[i][0],     # x 좌표 이동
                                 current.y + self.motion[i][1],     # y 좌표 이동
                                 current.cost + self.motion[i][2] * (1 + is_turned),  # 이동 비용에 방향 전환 비용 추가
                                 c_id,                             # 현재 노드의 인덱스를 부모로 설정
                                 now_vector)                       # 현재 이동 방향 벡터
                
                # 이웃 노드의 그리드 인덱스 계산 (해당 노드를 고유하게 식별)
                n_id = self.calc_grid_index(node)

                # 탐색할 수 없는 경우. 
                if (not is_starting) and (not self.verify_node(node)):
                    continue

                # 이미 탐색한 경우.
                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # 새로운 노드를 발견했습니다

                else:
                    if open_set[n_id].cost > node.cost:
                        # 지금까지는 이 길이 최선이다. 저장. 
                        open_set[n_id] = node

                is_starting = False


        rx, ry, tpx, tpy, tvec_x, tvec_y = self.calc_final_path(goal_node, closed_set)


        # grid position to map position
        for i in range(len(tpx)):
            tpx[i] = (tpx[i] * self.map_resolution) + self.map_origin[0]
            tpy[i] = (tpy[i] * self.map_resolution) + self.map_origin[1]

        print(tpx, tpy, tvec_x, tvec_y)

        return rx, ry, tpx, tpy, tvec_x, tvec_y

    def calc_final_path(self, goal_node, closed_set):
        """
        목표 노드에서 시작 노드까지의 최종 경로를 생성하는 함수.

        A* 알고리즘에서 목표 지점(goal_node)에 도달한 후, 경로 탐색을 완료한 클로즈드 셋(closed_set)을
        사용하여 목표 노드에서부터 시작 노드까지의 경로를 역으로 추적하여 최종 경로를 생성합니다.

        매개변수:
        - goal_node (Node): 경로 탐색이 완료된 목표 지점의 노드.
        - closed_set (dict): 탐색이 완료된 노드들이 저장된 딕셔너리 (키는 노드 인덱스, 값은 노드 객체).

        반환값:
        - rx (list): 최종 경로의 x 좌표 리스트 (그리드 상의 좌표).
        - ry (list): 최종 경로의 y 좌표 리스트 (그리드 상의 좌표).
        - tpx (list): 경로에서 방향이 바뀐 지점의 x 좌표 리스트 (실제 맵 좌표).
        - tpy (list): 경로에서 방향이 바뀐 지점의 y 좌표 리스트 (실제 맵 좌표).
        - tvec_x (list): 각 방향이 바뀐 지점에서의 x 방향 벡터 리스트.
        - tvec_y (list): 각 방향이 바뀐 지점에서의 y 방향 벡터 리스트.

        기능:
        - 목표 노드에서부터 부모 노드를 따라가며 시작 노드까지의 경로를 역으로 추적.
        - 추적된 경로는 목표 지점에서 시작 지점으로 가는 역순이므로, 결과적으로 경로를 반전하여 반환.
        - 경로 상에서 방향이 바뀌는 지점을 기록하여, 로봇이 회전해야 하는 지점을 추출.
        - 추출된 방향 벡터와 회전 지점은 로봇의 방향 전환에 사용될 수 있음.
        """
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [self.calc_grid_position(goal_node.y, self.min_y)]
        tpx, tpy = [], []
        tvec_x, tvec_y = [], []
        
        parent_index = goal_node.parent_index
        now_node = goal_node
        before_vector = (0, 0)
        now_vector = (0, 0)

        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))

            is_turned = 0
            now_vector = (n.x - now_node.x, n.y - now_node.y)
            is_turned = now_vector != before_vector
        
            if is_turned:
                tpx.append(now_node.x)
                tpy.append(now_node.y)
                tvec_x.append(now_node.vector[0])
                tvec_y.append(now_node.vector[1])

            parent_index = n.parent_index
            now_node = n
            before_vector = now_vector
        
        return rx, ry, tpx[::-1], tpy[::-1], tvec_x[::-1], tvec_y[::-1]

    @staticmethod
    def calc_heuristic(n1, n2):
        """
        두 노드 간의 휴리스틱 값을 계산하는 함수.

        매개변수:
        - n1: 첫 번째 노드 객체 (현재 노드).
        - n2: 두 번째 노드 객체 (목표 노드).

        반환값:
        - d (float): 두 노드 간의 유클리드 거리로 계산된 휴리스틱 값.

        기능:
        - 두 노드 간의 유클리드 거리(Euclidean distance)를 계산하여 휴리스틱 값으로 사용.
        - 거리 계산 공식은 math.hypot()을 사용하여, 피타고라스 정리에 따라 두 점 사이의 직선 거리를 구함.
        - `w`는 휴리스틱 값에 가중치를 부여하는 역할을 하며, 기본값은 1.0으로 설정되어 있음.
        - 반환된 값은 A* 알고리즘에서 탐색 우선순위를 결정하는 데 사용됨.
        """
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d
    
    @staticmethod
    def calc_manhattan(n1, n2):
        """
        두 노드 간의 맨해튼 거리를 계산하는 함수.

        맨해튼 거리(Manhattan distance)는 두 점 간의 직선 거리가 아니라, 
        수직 및 수평으로만 이동할 수 있을 때의 거리를 의미합니다.
        A* 알고리즘에서 휴리스틱 값을 계산할 때 자주 사용됩니다.

        매개변수:
        - n1: 첫 번째 노드 객체 (현재 노드).
        - n2: 두 번째 노드 객체 (목표 노드).

        반환값:
        - d (float): 두 노드 간의 맨해튼 거리.

        기능:
        - 두 노드 간의 x 좌표 차이와 y 좌표 차이를 각각 절대값으로 계산하고, 이를 더해 맨해튼 거리를 구함.
        - 맨해튼 거리 계산 공식: d = |n1.x - n2.x| + |n1.y - n2.y|
        - 직교 그리드 상에서, 수직 및 수평으로만 이동할 수 있을 때 사용되는 거리 계산 방법으로, 
        유클리드 거리 대신 맨해튼 거리를 사용하여 더 적합한 경로를 찾을 수 있음.
        """
        d = abs(n1.x - n2.x) + abs(n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        그리드 인덱스를 실제 좌표로 변환하는 함수.

        이 함수는 A* 탐색에서 사용된 그리드 인덱스를 실제 맵 상의 좌표로 변환합니다. 
        그리드 인덱스에 해당하는 위치를 실제 맵 좌표계에서의 값으로 계산할 수 있습니다.

        매개변수:
        - index (int): 그리드 상의 인덱스 (x 또는 y 좌표의 인덱스).
        - min_position (float): 해당 그리드의 최소 좌표 (x 또는 y의 최소값).

        반환값:
        - position (float): 그리드 인덱스에 해당하는 실제 좌표.

        기능:
        - 그리드 인덱스를 그리드 해상도(self.resolution)와 최소 좌표(min_position)를 기반으로 실제 좌표로 변환.
        - 변환 공식: 실제 좌표 = (그리드 인덱스 * 해상도) + 최소 좌표.
        - 이를 통해 그리드 상의 인덱스 값을 맵 좌표계에서의 실제 위치로 변환할 수 있습니다.
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        """
        주어진 실제 좌표를 그리드 인덱스로 변환하는 함수.

        이 함수는 실제 좌표값을 A* 탐색에서 사용할 수 있는 그리드 인덱스로 변환합니다.
        실제 좌표를 그리드 크기(`self.resolution`)로 나누어 해당 좌표가 그리드 상에서 어느 위치에 있는지를 계산합니다.

        매개변수:
        - position (float): 실제 좌표값 (x 또는 y 좌표).
        - min_pos (float): 맵의 최소 x 또는 y 좌표.

        반환값:
        - index (int): 주어진 실제 좌표에 해당하는 그리드 상의 인덱스.

        기능:
        - 실제 좌표에서 최소 좌표(min_pos)를 뺀 값을 그리드 해상도(self.resolution)로 나누어 그리드 인덱스를 계산합니다.
        - 예를 들어, 그리드 해상도가 1m인 경우, 3.5m의 위치는 그리드 상에서 3번째 칸에 해당하게 됩니다.
        """
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        """
        노드의 그리드 인덱스를 계산하는 함수.

        이 함수는 주어진 노드의 x, y 좌표를 기반으로, 노드의 고유한 그리드 인덱스를 계산합니다.
        이는 A* 알고리즘에서 탐색된 노드를 고유하게 식별할 수 있도록 하기 위해 사용됩니다.

        매개변수:
        - node (Node): 그리드 상의 특정 위치를 나타내는 노드 객체.

        반환값:
        - grid_index (int): 노드의 x, y 좌표를 기반으로 계산된 고유한 그리드 인덱스.

        기능:
        - x 좌표와 y 좌표를 기반으로 2차원 좌표를 1차원으로 변환하여 고유한 인덱스를 생성합니다.
        - 인덱스는 노드의 x 좌표와 y 좌표를 이용해 `grid_index = (y * 그리드 너비) + x`의 형태로 계산됩니다.
        - 이를 통해 각 노드를 고유하게 식별하고, A* 알고리즘에서 노드 간의 탐색 상태를 관리할 수 있게 합니다.
        """
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        """
        노드가 유효한지 확인하는 함수.

        이 함수는 주어진 노드가 유효한 그리드 내에 있는지, 그리고 장애물에 속하지 않는지 확인합니다.
        A* 알고리즘에서 경로 탐색 중, 노드가 장애물 위에 있거나 그리드의 범위를 벗어난 경우, 
        해당 노드를 탐색하지 않도록 하기 위해 사용됩니다.

        매개변수:
        - node (Node): 검사할 노드 객체.

        반환값:
        - (bool): 노드가 유효한 경우 True, 그렇지 않은 경우 False.

        기능:
        - 노드의 x, y 좌표가 그리드의 범위를 벗어나는지 확인.
        - 해당 노드가 장애물 맵(obstacle_map)에 있는지 확인.
        - 유효한 노드일 경우 True를 반환하고, 그리드 범위 외에 있거나 장애물 위에 있는 경우 False를 반환.
        """
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False


        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        """
        장애물 맵을 생성하는 함수.

        이 함수는 장애물 좌표 리스트(ox, oy)를 기반으로, 그리드 상에서 각 장애물의 위치를 기록한
        장애물 맵(obstacle_map)을 생성합니다. 장애물 맵은 A* 알고리즘이 경로 탐색 중 장애물을 피할 수 있도록
        하기 위해 사용됩니다.

        매개변수:
        - ox (list): 장애물의 x 좌표 리스트.
        - oy (list): 장애물의 y 좌표 리스트.

        기능:
        - 그리드 상에서 장애물이 위치할 수 있는 최소/최대 x, y 값을 계산하여 그리드의 크기를 결정합니다.
        - 각 그리드 셀에 장애물이 있는지 없는지를 확인하고, 장애물 맵(obstacle_map)을 생성합니다.
        - A* 탐색에서 장애물과의 충돌을 피하기 위해 해당 맵을 참조합니다.
        """
        print("Calc Obstacle...!")

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)
        
        # 장애물 맵 초기화
        obstacle_map = np.zeros((self.x_width, self.y_width), dtype=bool)
        
        # 장애물 좌표를 NumPy 배열로 변환
        ox = np.array(ox)
        oy = np.array(oy)
        
        # 인덱스 계산 (벡터화 연산)
        ix = np.round((ox - self.min_x) / self.resolution).astype(int)
        iy = np.round((oy - self.min_y) / self.resolution).astype(int)
                
        # 인덱스 경계 제한
        ix = np.clip(ix, 0, self.x_width - 1)
        iy = np.clip(iy, 0, self.y_width - 1)
        
        # 장애물 위치 설정
        obstacle_map[ix, iy] = True
        
        # 로봇 반경을 고려한 패딩 적용
        distance_map = distance_transform_edt(~obstacle_map) * self.resolution  # 장애물과 장애물이 아닌 셀 간의 거리 계산
        self.obstacle_map = distance_map <= self.rr
        
        print("Obstacle map calculated with robot radius.")
        # obstacle map generation
        # self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        # for ix in range(self.x_width):
        #     x = elf.calc_grid_position(ix, self.min_x)
        #     for iy in range(self.y_width):
        #         y = self.calc_grid_position(iy, self.min_y)
        #         for iox, ioy in zip(ox, oy):
        #             d = math.hypot(iox - x, ioy - y)
        #             if d <= self.rr:
        #                 self.obstacle_map[ix][iy] = True
        #                 break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            # [-1, -1, math.sqrt(2)],
            # [-1, 1, math.sqrt(2)],
            # [1, -1, math.sqrt(2)],
            # [1, 1, math.sqrt(2)]
        ]

        return motion

def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 0  # [m]
    sy = 0  # [m]
    gx = -0.16069  # [m]
    gy = -1.9620166  # [m]
    grid_size = 0.7  # [m]
    robot_radius = 0.3  # [m]
    padding = 3


    a_star = AStarPlanner(grid_size, robot_radius, padding)

    rx, ry, tpx, tpy, tvec = a_star.planning(sx, sy, gx, gy)


if __name__ == '__main__':
    main()



    