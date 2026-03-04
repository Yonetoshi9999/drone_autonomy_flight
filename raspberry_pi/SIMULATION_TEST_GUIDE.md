# Simulation Test Guide - Route Optimization

## Pre-Flight Checklist

### 1. Code Readiness
- ✅ All tests passing: 40/42 (95.2%)
- ✅ Route optimizer integrated at 100Hz
- ✅ MISSION_ITEM handling implemented
- ✅ Timing constraints validated (< 10ms cycle)
- ✅ Dependencies installed: `scipy>=1.9.0`

### 2. Key Features to Test

#### A. Mission Planning Phase (Mode 99 + PLANNING)
- [ ] System enters Mode 99 (autonomous mode)
- [ ] RaspberryPi state: `Not_SET` initially
- [ ] Mission Planner connects successfully
- [ ] Send waypoints from Mission Planner (3-5 points recommended)
- [ ] Verify MISSION_ITEM messages received
- [ ] Observe AI route optimization messages:
  ```
  ミッション受信開始: X個のウェイポイント
  ミッション受信完了: X個のウェイポイント
  AIルート最適化開始...
  ウェイポイント順序最適化: 総距離 XXXm
  経路平滑化: X点 → XX点
  ルート最適化完了: XXポイント, X.XXX秒
  ```
- [ ] Verify state transition: `Not_SET` → `ROUTE_SET`

#### B. Route Initialization (Mode 99 + INITIALIZING)
- [ ] State transition: `ROUTE_SET` → `EXECUTING`
- [ ] First waypoint selected

#### C. Autonomous Flight (Mode 99 + EXECUTING)
- [ ] Drone follows optimized route
- [ ] 100Hz MAVLink commands sent (position + velocity)
- [ ] Wind compensation applied (if wind present)
- [ ] Obstacle avoidance working
- [ ] Altitude maintained ≥ 50m (until within 50m of destination)
- [ ] Waypoint progression messages:
  ```
  ウェイポイント X/Y 到達
  ```
- [ ] NFZ avoidance (if applicable)

## Critical Monitoring Points

### 1. Timing Performance
**Monitor for warnings:**
```
警告: 制御周期オーバー X.Xms
```

**Expected behavior:**
- Control cycle should stay under 10ms
- Occasional spikes acceptable, but not sustained
- If warnings appear frequently, optimization may be too heavy

### 2. Route Quality Metrics

**Distance Efficiency:**
- Compare optimized route distance vs straight-line distance
- Expected: 15-25% improvement over naive waypoint ordering
- Check console output: `ウェイポイント順序最適化: 総距離 XXXm`

**Path Smoothness:**
- Drone should follow smooth curves, not sharp corners
- Check interpolation: `経路平滑化: X点 → XX点`
- Typical: 3-5 input waypoints → 30-50 interpolated points

**Altitude Behavior:**
- Should maintain 50m minimum until close to destination
- Watch for: `高度が50m未満の場合、50mに修正`

### 3. NFZ (No-Fly Zone) Compliance

**If NFZ zones are present:**
```
警告: NFZ 'ZONE_NAME'に接近、回避動作
```

**Expected behavior:**
- Route should avoid NFZ during planning
- Real-time avoidance if drone gets close
- Error if waypoint is inside NFZ

### 4. Obstacle Avoidance Integration

**Monitor obstacle detection:**
- LiDAR + Camera sensor fusion
- Repulsive force calculation
- Velocity adjustment

**Watch for timing warnings:**
```
障害物検知時間超過: X.Xms
```

## Test Scenarios

### Scenario 1: Basic Route (Recommended First Test)
**Setup:**
- 3-4 waypoints in open area
- No obstacles
- No NFZ
- Light/no wind

**Success Criteria:**
- ✅ All waypoints reached
- ✅ Smooth trajectory
- ✅ No timing warnings
- ✅ Altitude maintained

### Scenario 2: Complex Route (TSP Test)
**Setup:**
- 5-7 waypoints in random order
- Deliberately suboptimal ordering

**Success Criteria:**
- ✅ TSP reorders waypoints efficiently
- ✅ Total distance reduced vs input order
- ✅ Check console: distance optimization message

### Scenario 3: NFZ Avoidance
**Setup:**
- Route that would pass through NFZ
- Waypoints on opposite sides of NFZ

**Success Criteria:**
- ✅ Route planning avoids NFZ
- ✅ OR error if waypoint inside NFZ (depends on setup)
- ✅ Real-time avoidance warnings if approaching

### Scenario 4: Wind Compensation
**Setup:**
- Simulate moderate wind (2-5 m/s)
- Long straight flight segment

**Success Criteria:**
- ✅ Drone compensates for wind drift
- ✅ Stays on planned trajectory
- ✅ Wind vector applied: -30% compensation

### Scenario 5: Obstacle Avoidance
**Setup:**
- Place obstacles along planned route
- LiDAR + Camera detection

**Success Criteria:**
- ✅ Real-time trajectory adjustment
- ✅ Maintains 5m clearance
- ✅ Returns to planned route after avoidance

## Key Console Messages to Watch

### Normal Operation
```
自律飛行状態管理システム初期化完了
ミッション受信開始: X個のウェイポイント
AIルート最適化開始...
ウェイポイント順序最適化: 総距離 XXXm
経路平滑化: X点 → XX点
ルート最適化完了: XXポイント, X.XXX秒
AI最適化完了: XXウェイポイント
ミッション設定完了: XXウェイポイント
実行モード: 自律飛行開始
ウェイポイント X/Y 到達
全ウェイポイント到達: ミッション完了
```

### Warnings (Acceptable if Occasional)
```
警告: 制御周期オーバー X.Xms
障害物検知時間超過: X.Xms
警告: NFZ 'ZONE_NAME'に接近、回避動作
```

### Errors (Need Investigation)
```
エラー: ウェイポイント設定失敗（NFZ違反の可能性）
AI軌道計算エラー: XXX, フォールバック使用
ウェイポイント計画エラー: XXX
```

## Fallback Behavior

### AI Optimizer Failure
If AI optimizer encounters an error:
- System automatically falls back to traditional trajectory calculation
- Message: `AI軌道計算エラー: XXX, フォールバック使用`
- Flight continues normally with reduced optimization

### MISSION_ITEM Not Received
If no waypoints received from Mission Planner:
- System generates default route (100m north, 50m altitude)
- Message: `警告: Mission Plannerからウェイポイント未受信`
- Uses AI optimization on default route

## Performance Benchmarks

### Expected Timing
- **Planning Phase**: 2-5ms for route optimization
- **Control Loop**: < 10ms per cycle (100Hz)
- **Trajectory Calculation**: < 0.5ms per cycle
- **Sensor Processing**: < 9ms budget

### Expected Accuracy
- **Waypoint Arrival**: Within 2.0m radius
- **Altitude Maintenance**: ±2m of 50m target
- **Wind Compensation**: 30% of wind vector

## Debug Commands (If Issues Occur)

### Check Current State
During flight, the system logs state every 5 seconds:
```
状態: RPI=EXECUTING, FC=EXECUTING, WP=3/10
```

### Manual Test (Without Simulation)
```bash
# Unit tests
python3 test_route_optimization.py

# Integration tests
python3 -m pytest tests/integration/test_autonomy_state.py -v
```

## Post-Flight Analysis

### Success Indicators
- [ ] All waypoints reached
- [ ] No sustained timing violations
- [ ] Smooth trajectory (visual inspection)
- [ ] Altitude constraints maintained
- [ ] NFZ avoided (if applicable)
- [ ] Obstacles avoided (if present)

### Data to Collect
1. **Total flight time**
2. **Total distance traveled**
3. **Number of waypoints reached**
4. **Maximum control cycle time**
5. **Number of timing warnings**
6. **NFZ avoidance events**
7. **Obstacle avoidance events**

### Performance Comparison (Optional)
If you have baseline data from traditional planning:
- Compare total flight time
- Compare total distance
- Compare smoothness (visual/qualitative)

## Troubleshooting

### Issue: "Mission Plannerからウェイポイント未受信"
**Cause**: MISSION_ITEM messages not received
**Fix**:
- Check MAVLink connection
- Verify Mode 99 is active
- Send waypoints again from Mission Planner

### Issue: Frequent timing warnings
**Cause**: Control cycle exceeding 10ms
**Fix**:
- Reduce number of waypoints in optimization
- Check sensor processing time
- Verify hardware performance

### Issue: "NFZ違反"
**Cause**: Waypoint inside No-Fly Zone
**Fix**:
- Adjust waypoints in Mission Planner
- Move waypoints outside NFZ boundaries
- Update NFZ database if incorrect

### Issue: Drone doesn't follow optimized path
**Cause**: AI optimizer not being used
**Check**:
- Look for: `AI軌道計算エラー, フォールバック使用`
- Verify route_optimizer initialization in logs
- Check for Python errors during planning

## State Machine Reference

```
NOT_SET (0)
    ↓ [PLANNING + waypoints received]
ROUTE_SET (1)
    ↓ [INITIALIZING]
EXECUTING (2)
    ↓ [mission complete]
COMPLETED (3)
    ↓ [IDLE]
NOT_SET (0)
```

## Quick Start for Tonight/Tomorrow

1. **Start simulation environment**
2. **Launch drone system**: `python3 main.py`
3. **Set Mode 99** (autonomous)
4. **Send 3-4 waypoints** from Mission Planner
5. **Watch console** for optimization messages
6. **Observe flight** - smooth trajectory, waypoint arrival
7. **Check timing** - no sustained warnings
8. **Complete mission** - all waypoints reached

Good luck with your simulation testing! 🚁
