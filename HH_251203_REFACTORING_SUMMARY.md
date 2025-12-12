# camping_cart_ws 리팩토링 요약 - HH_251203

**작성일**: December 3, 2025  
**작업자**: Hong (HH)  
**요약**: Autoware 스타일 map subsystem 생성 및 planning 패키지 리팩토링

---

## 📋 주요 작업 내용

### 1. 새 패키지: `camping_cart_map` 생성 (HH_251203)

**목적**: Autoware Universe를 참고한 map subsystem 분리

**구조**:
```
camping_cart_map/                           # 새로 생성
├── CMakeLists.txt                         # 빌드 설정
├── package.xml                            # 메타데이터
├── README.md                              # 사용법 및 상태
├── include/camping_cart_map/
│   ├── lanelet2_map_loader.hpp            # OSM 로더 (HH_251203)
│   ├── map_projector_loader.hpp           # 좌표 변환 (HH_251203)
│   ├── map_tf_generator.hpp               # TF 생성 (HH_251203)
│   └── lanelet2_map_visualization.hpp     # 시각화 (HH_251203, 준비중)
├── src/
│   ├── lanelet2_map_loader.cpp            # 로더 구현 (HH_251203)
│   ├── lanelet2_map_loader_node.cpp       # 로더 노드 (HH_251203)
│   ├── map_projector_loader.cpp           # 프로젝터 (HH_251203)
│   ├── map_projector_loader_node.cpp      # 프로젝터 노드 (HH_251203)
│   ├── map_tf_generator.cpp               # TF 생성 (HH_251203)
│   ├── map_tf_generator_node.cpp          # TF 노드 (HH_251203)
│   ├── lanelet2_map_visualization.cpp     # 시각화 (HH_251203, 준비중)
│   └── lanelet2_map_visualization_node.cpp # 시각화 노드 (HH_251203, 준비중)
├── launch/
│   └── map_system.launch.py               # 전체 시스템 (HH_251203)
└── config/
    ├── map_loader.param.yaml              # 로더 파라미터 (HH_251203)
    ├── map_projector.param.yaml           # 프로젝터 파라미터 (HH_251203)
    └── map_visualization.param.yaml       # 시각화 파라미터 (HH_251203)
```

**노드 설명**:

| 노드 | 역할 | 상태 |
|------|------|------|
| `lanelet2_map_loader_node` | OSM 파일 로드 | 준비중 |
| `map_projector_loader_node` | WGS84→ENU 변환 | 준비중 |
| `map_tf_generator_node` | TF broadcast | 준비중 |
| `lanelet2_map_visualization_node` | RViz 마커 | 준비중 |

---

### 2. 패키지 이름 변경: `camping_cart_navigation` → `camping_cart_planning` (HH_251203)

**이유**: 
- Lanelet2 로더를 camping_cart_map으로 완전 이관
- 관심사 분리: map subsystem vs global planner
- Autoware 구조 체계화

**변경 사항**:

| 항목 | 이전 | 현재 |
|------|------|------|
| 패키지명 | camping_cart_navigation | camping_cart_planning |
| 포함 기능 | 맵 로더 + 계획 | 계획만 |
| 의존성 | 복잡 (로더 내장) | 단순 (ROS2 + Lanelet2) |
| 빌드 | camping_cart_map 필요 | 독립 빌드 가능 |

**제거된 파일**:
```
-src/lanelet2_loader_node.cpp
-src/lanelet2_loader.cpp
-include/camping_cart_navigation/lanelet2_loader.hpp
-scripts/find_map_origin.py
```

**추가된 파일**:
```
+README.md (HH_251203)
```

**수정된 파일**:
```
src/global_planner_node.cpp      # 단순화 (HH_251203)
launch/global_planner.launch.py  # 리팩토링 (HH_251203)
CMakeLists.txt                   # 의존성 제거 (HH_251203)
package.xml                      # 패키지명 변경 (HH_251203)
```

---

## 📦 파일별 변경 사항

### Header Files (camping_cart_map)

#### `lanelet2_map_loader.hpp` (HH_251203)
- 목적: OSM 파일 로드
- 주요 클래스: `Lanelet2MapLoader`
- 메서드: `load()`, `laneletCount()`, `getMapStats()`
- 상태: 구조 완성, 빌드 진행중

#### `map_projector_loader.hpp` (HH_251203)
- 목적: WGS84 → ENU 좌표 변환
- 주요 클래스: `MapProjectorLoader`
- 구조체: `MapProjectorConfig`
- 메서드: `getProjector()`, `forward()`, `inverse()`
- 상태: 구조 완성, 빌드 진행중

#### `map_tf_generator.hpp` (HH_251203)
- 목적: TF 변환 broadcast
- 주요 클래스: `MapTfGenerator`
- 메서드: `broadcastTransform()`, `broadcastMapOrigin()`
- 상태: 구조 완성, 빌드 진행중

#### `lanelet2_map_visualization.hpp` (HH_251203)
- 목적: RViz MarkerArray 생성
- 주요 클래스: `Lanelet2MapVisualization`
- 메서드: `visualize()`, `createCenterlineMarkers()`, `createBoundaryMarkers()`, 등
- 상태: 구조 완성, 빌드 준비중

### Node Files

#### `lanelet2_map_loader_node.cpp` (HH_251203)
- 역할: OSM 맵 로드 및 통계 출력
- 파라미터: `map_path`, `remove_speed_bumps`
- 상태: 구현 완료, 빌드 진행중

#### `map_projector_loader_node.cpp` (HH_251203)
- 역할: 프로젝터 초기화 및 `/map/projection` 발행
- 파라미터: `origin_lat`, `origin_lon`, `origin_alt`
- 발행 토픽: `/map/projection` (std_msgs/String - JSON)
- 상태: 구현 완료, 빌드 진행중

#### `map_tf_generator_node.cpp` (HH_251203)
- 역할: 정적 TF 변환 broadcast
- 파라미터: `parent_frame`, `child_frame`, `tf_x`, `tf_y`, `tf_z`, `tf_roll`, `tf_pitch`, `tf_yaw`
- 상태: 구현 완료, 빌드 진행중

#### `lanelet2_map_visualization_node.cpp` (HH_251203)
- 역할: 맵 시각화 마커 생성 및 발행
- 파라미터: `map_path`, `origin_lat`, `origin_lon`, `frame_id`
- 발행 토픽: `/map/centerlines`, `/map/bounds`, `/map/lanelet_ids`
- 상태: 단순화 버전 구현, 빌드 진행중

#### `global_planner_node.cpp` (HH_251202 → HH_251203)
- 역할: RViz 클릭 → 경로 계획
- 구독 토픽: `/goal_pose` (RViz 2D Nav Goal)
- 발행 토픽: `/planning/global_path` (nav_msgs/Path)
- 상태: 간단한 버전 완료 (실제 경로 계획은 TODO)

---

## 🔧 빌드 상태

### ✅ 완성됨
- **camping_cart_planning**
  - 독립적 빌드 성공 ✓
  - 모든 파일 정리됨 ✓
  - README 작성됨 ✓

### ⚠️ 진행중 (camping_cart_map)
- 패키지 구조 완성 ✓
- 모든 헤더/소스 파일 생성 ✓
- **빌드 이슈**: Lanelet2 네임스페이스 일관성 필요
  - 문제: `lanelet::` vs `lanelet2::` 혼재
  - 영향: 4개 노드 컴파일 오류
  - 예상 해결: Lanelet2 헤더 경로 검증

---

## 📊 아키텍처 다이어그램 (HH_251203)

```
┌─────────────────────────────────────┐
│  camping_cart_map (Map Subsystem)   │  [HH_251203] 새로 생성
├─────────────────────────────────────┤
│  lanelet2_map_loader_node           │  OSM → LaneletMap
│  map_projector_loader_node          │  WGS84 → ENU
│  map_tf_generator_node              │  world → map TF
│  lanelet2_map_visualization_node    │  RViz markers
└──────────────┬──────────────────────┘
               │ Topics: /map/*
┌──────────────▼──────────────────────┐
│ camping_cart_planning (Planner)     │  [HH_251203] 리팩토링
├─────────────────────────────────────┤
│  global_planner_node                │  /goal_pose → /planning/global_path
│  (단순화 버전, 실제 라우팅 TODO)    │
└─────────────────────────────────────┘
```

---

## 📝 문서 (HH_251203)

### 생성된 README
- `/home/hong/camping_cart_ws/src/camping_cart_map/README.md`
- `/home/hong/camping_cart_ws/src/camping_cart_planning/README.md`

### 주석 추가
- 모든 헤더 파일: HH_251203 주석 추가
- 모든 소스 파일: HH_251203 주석 추가
- 모든 노드 파일: HH_251203 주석 추가
- CMakeLists.txt & package.xml: HH_251203 주석 추가

---

## 🚀 다음 단계

### 1. camping_cart_map 빌드 완료 (우선순위 높음)
- [ ] Lanelet2 네임스페이스 일관성 확인
- [ ] 헤더 파일 경로 검증
- [ ] 4개 노드 컴파일 오류 해결
- [ ] `colcon build --packages-select camping_cart_map` 성공

### 2. camping_cart_planning과 통합
- [ ] camping_cart_map 의존성 추가
- [ ] global_planner_node에서 실제 라우팅 구현
- [ ] launch 파일 통합

### 3. RViz 통합
- [ ] camping_cart_planning용 RViz config 생성
- [ ] 맵 시각화 확인
- [ ] 경로 시각화 확인

### 4. 테스트
- [ ] 단위 테스트 추가
- [ ] 통합 테스트
- [ ] RViz 상에서 엔드-투-엔드 테스트

---

## 📚 참고

- **Autoware Universe**: https://github.com/autowarefoundation/autoware.universe
- **Lanelet2**: https://github.com/fzi-forschungszentrum-informatik/Lanelet2
- **ROS2 Humble**: https://docs.ros.org/en/humble/

---

**작업 완료 날짜**: December 3, 2025  
**마지막 수정**: HH_251203  
**상태**: 진행중 (camping_cart_map 빌드 대기)
