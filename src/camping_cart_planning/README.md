# camping_cart_planning — Global Path Planner

**작성일**: HH_251202 (Dec 2, 2025)  
**리팩토링**: HH_251203 (Dec 3, 2025)

## 개요

Lanelet2 routing을 사용한 자율 캠핑카 경로 계획 모듈입니다.

### 주요 기능

- **RViz 클릭 기반 목표 설정**: RViz의 "2D Nav Goal" 도구로 목표 설정
- **Lanelet2 라우팅**: 최단 경로 자동 계산
- **동적 목표 업데이트**: 파라미터 또는 RViz 클릭으로 실시간 목표 변경
- **자동 귀차 기능**: 타임아웃 후 시작점으로 자동 복귀 (향후)

## 패키지 구조

```
camping_cart_planning/
├── CMakeLists.txt                      # 빌드 설정
├── package.xml                         # 패키지 메타데이터
├── include/camping_cart_planning/      # 헤더 파일 (향후 추가)
├── src/
│   └── global_planner_node.cpp        # 전역 경로 계획 노드
├── launch/
│   └── global_planner.launch.py       # 런칭 파일
└── config/
    └── rviz_planner.rviz              # RViz 설정 (향후)
```

## 빌드 상태 (HH_251203)

### ✅ 완성됨
- camping_cart_navigation → camping_cart_planning 으로 패키지 이름 변경
- Lanelet2 로더 제거 (camping_cart_map으로 이관)
- global_planner_node.cpp 단순화 및 재작성
- 독립적 빌드 가능 (camping_cart_map 의존성 제거)

### ⚠️향후 개선
- camping_cart_map과의 통합
- 실제 경로 계획 로직 구현
- RViz config 추가
- 타임아웃 귀차 기능

## 설치 및 빌드

```bash
cd /home/hong/camping_cart_ws
colcon build --packages-select camping_cart_planning

# 전체 빌드
colcon build
```

## 사용법

### 전역 경로 계획 실행

```bash
ros2 launch camping_cart_planning global_planner.launch.py \
  map_path:=/path/to/map.osm \
  origin_lat:=36.12491471 \
  origin_lon:=126.77473954
```

### 노드 실행

```bash
ros2 run camping_cart_planning global_planner_node --ros-args \
  -p map_path:=/path/to/map.osm \
  -p origin_lat:=36.12491471 \
  -p origin_lon:=126.77473954 \
  -p goal_lane_id:=-1
```

### RViz에서 목표 설정

1. RViz 실행
2. Fixed Frame을 `map`으로 설정
3. 도구 모음에서 "2D Nav Goal" 선택
4. 지도 위에서 클릭

## 구독 토픽

- `/goal_pose` (geometry_msgs/PoseStamped): RViz 2D Nav Goal

## 발행 토픽

- `/planning/global_path` (nav_msgs/Path): 계획된 경로

## 파라미터

- `map_path`: Lanelet2 OSM 파일 경로 (기본값: "")
- `origin_lat`: WGS84 위도 (기본값: 37.0)
- `origin_lon`: WGS84 경도 (기본값: 127.0)
- `goal_lane_id`: 목표 차선 ID (기본값: -1)

## 변경 이력 (HH_251203)

### 주요 리팩토링

1. **패키지 이름 변경**
   - `camping_cart_navigation` → `camping_cart_planning`
   - 이유: 향후 camping_cart_map 패키지와의 분리 및 계획 기능만 담당

2. **Lanelet2 로더 제거**
   - 기존: planning 패키지 내 로더 포함
   - 변경: camping_cart_map 패키지로 완전 이관
   - 장점: 관심사 분리 (map subsystem vs planning)

3. **파일 정리**
   - 제거된 파일:
     - `src/lanelet2_loader_node.cpp`
     - `src/lanelet2_loader.cpp`
     - `include/camping_cart_planning/lanelet2_loader.hpp`
     - `scripts/find_map_origin.py`

4. **global_planner_node.cpp 단순화**
   - 구 버전: 복잡한 lanelet2 로더 포함
   - 신 버전: 간단한 RViz 구독만 포함
   - `/goal_pose` 구독 → `/planning/global_path` 발행

5. **CMakeLists.txt 업데이트**
   - camping_cart_map 의존성 제거
   - 필수 의존성만 유지 (ROS2 메시지, Lanelet2 routing)

## 전체 아키텍처 (HH_251203)

```
┌─────────────────────────────────────┐
│   camping_cart_map (Map Subsystem)  │
├─────────────────────────────────────┤
│ • lanelet2_map_loader_node          │
│ • map_projector_loader_node         │
│ • map_tf_generator_node             │
│ • lanelet2_map_visualization_node   │
└──────────────────┬──────────────────┘
                   │ (Map topics)
┌──────────────────▼──────────────────┐
│ camping_cart_planning (Planner)     │
├─────────────────────────────────────┤
│ • global_planner_node               │
│   ├─ 구독: /goal_pose               │
│   └─ 발행: /planning/global_path    │
└─────────────────────────────────────┘
```

## 향후 작업

1. camping_cart_map 빌드 완성
2. global_planner_node에서 실제 경로 계획 구현
3. 파라미터 기반 사이트 매핑
4. 자동 귀차 타이머 추가
5. RViz visualization 통합

---

**참고**: HH_251202부터 HH_251203까지 진행된 리팩토링에 대한 자세한 내용은 각 소스 파일의 주석을 참고하세요.
