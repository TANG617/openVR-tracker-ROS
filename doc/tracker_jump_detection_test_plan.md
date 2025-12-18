# HTC Tracker 数据跳跃检测与处理 测试方案

## 一、功能概述

HTC Tracker 驱动模块实现了数据跳跃检测和处理机制，用于：
- 检测信号异常导致的位置突变（跳跃）
- 在检测到跳跃后自动停止数据输出，保护下游系统（机器人EE pose）
- 当数据恢复稳定后自动恢复输出

## 二、参数配置说明

### 2.1 跳跃检测器（JumpDetector）参数

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `enable_jump_detection` | true | - | 是否启用跳跃检测 |
| `jump_speed_limit` | 5.0 | m/s | 速度超过此值判定为跳跃 |
| `recovery_window_size` | 20 | 帧 | 恢复所需的稳定帧数 |
| `stability_threshold` | 0.0001 | m² | 位置方差阈值（标准差约1cm） |

### 2.2 滤波器（Filter）参数

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `enable` | true | - | 是否启用滤波 |
| `normal_speed_limit` | 2.0 | m/s | 速度限幅上限 |
| `smoothing_alpha` | 0.3 | - | 指数平滑系数（越小越平滑） |

### 2.3 参数调优指南

**jump_speed_limit（跳跃速度阈值）**
- 值越低：更敏感，可能误触发
- 值越高：更宽容，可能漏检
- 建议：机器人应用 4-6 m/s，人机交互 6-8 m/s

**recovery_window_size（恢复窗口大小）**
- 值越大：恢复越慢但越稳定
- 值越小：恢复越快但可能不稳定
- 建议：100Hz发布率下，15-25帧（150-250ms恢复时间）

**stability_threshold（稳定性阈值）**
- 这是位置方差阈值，单位是 m²
- 0.0001 m² 对应标准差约 1cm
- 0.0004 m² 对应标准差约 2cm
- 建议：高精度场景 0.0001，一般场景 0.0004

**smoothing_alpha（平滑系数）**
- 值越小：输出越平滑，但延迟越大
- 值越大：响应越快，但噪声越明显
- 建议：0.2-0.4

## 三、数据处理流程

```
┌─────────────┐
│ OpenVR数据  │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  Validator  │ ── 无效 ──► 丢弃（不输出）
│  数据验证   │
└──────┬──────┘
       │ 有效
       ▼
┌─────────────┐
│   Detector  │ ── 跳跃 ──► 进入锁定状态
│  跳跃检测   │ ── 锁定 ──► 等待恢复（不输出）
└──────┬──────┘
       │ 正常/恢复
       ▼
┌─────────────┐
│   Filter    │
│  滤波平滑   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  发布数据   │
│ calib_pose  │
└─────────────┘
```

## 四、测试环境准备

### 4.1 启动节点

```bash
# 终端1: 启动 SteamVR
ros2 launch tracker steamvr.launch.py

# 终端2: 启动 Tracker 节点（带调试日志）
ros2 launch tracker trackers.launch.py log_level:=debug
```

### 4.2 监控工具

```bash
# 终端3: 监控原始数据
ros2 topic echo /hal/tracker/htc/left/pose_raw

# 终端4: 监控校准后数据
ros2 topic echo /hal/tracker/htc/left/calib_target_pose

# 终端5: 监控发布频率
ros2 topic hz /hal/tracker/htc/left/calib_target_pose

# 终端6: 监控日志（关注跳跃检测相关信息）
# 日志会显示：
# - [JUMP] 检测到跳跃
# - [LOCKED] 锁定状态
# - [RECOVERED] 恢复正常
```

### 4.3 数据记录

```bash
# 记录测试数据用于后续分析
ros2 bag record \
  /hal/tracker/htc/left/pose_raw \
  /hal/tracker/htc/left/calib_target_pose \
  /hal/tracker/htc/right/pose_raw \
  /hal/tracker/htc/right/calib_target_pose \
  -o tracker_test_$(date +%Y%m%d_%H%M%S)
```

## 五、测试场景

### 测试 1: 正常运动基准测试

**目的**：确认正常运动下系统稳定工作

**步骤**：
1. 将 tracker 固定在稳定位置，静止 10 秒
2. 缓慢移动 tracker（约 0.5 m/s），持续 20 秒
3. 进行正常速度手部动作（约 1-2 m/s），持续 30 秒

**预期结果**：
- ✅ `calib_target_pose` 持续稳定输出
- ✅ 频率稳定在 100Hz（±2Hz）
- ✅ 日志无 JUMP/LOCKED 告警

**验证命令**：
```bash
ros2 topic hz /hal/tracker/htc/left/calib_target_pose
# 期望: average rate: ~100.0 Hz
```

---

### 测试 2: 跳跃触发测试

**目的**：验证跳跃检测能正确触发

**步骤**：
1. 保持 tracker 稳定 2 秒
2. **快速甩动** tracker（尽可能快，模拟信号跳跃）
3. 立即停止，观察日志

**预期结果**：
- ✅ 日志显示 `[JUMP DETECTED]` 信息
- ✅ `calib_target_pose` 立即停止发布
- ✅ `pose_raw` 继续正常发布
- ✅ 日志显示计算出的速度值

**触发条件**：移动速度 > 5.0 m/s（即 10ms 内移动 > 5cm）

---

### 测试 3: 自动恢复测试

**目的**：验证数据恢复机制

**步骤**：
1. 执行测试 2 触发跳跃
2. 立即将 tracker 放到**固定位置**，保持完全静止
3. 等待恢复（约 200ms = 20帧 @ 100Hz）

**预期结果**：
- ✅ 日志显示 `[LOCKED]` 状态，包含当前方差值
- ✅ 静止约 200ms 后，日志显示 `[RECOVERED]`
- ✅ `calib_target_pose` 自动恢复输出
- ✅ 恢复后位置准确，无跳变

**恢复时间计算**：
```
恢复时间 ≈ recovery_window_size / publish_rate
         = 20 / 100 = 0.2秒 = 200ms
```

---

### 测试 4: 遮挡/信号丢失测试

**目的**：模拟真实信号干扰场景

**步骤**：
1. 正常运动中，用手**完全遮挡** tracker 传感器（2-3 秒）
2. 移开遮挡物
3. 观察恢复行为

**预期结果**：
- ✅ 遮挡时 `pose_raw` 停止（Validator 返回 kInvalidPose）
- ✅ 信号恢复后，重新开始输出
- ✅ 无位置突变

---

### 测试 5: Base Station 视角边缘测试

**目的**：测试信号弱化区域的处理

**步骤**：
1. 将 tracker 移动到 Base Station 覆盖边缘
2. 在边缘区域快速来回移动
3. 移回正常覆盖区域

**预期结果**：
- ✅ 边缘区域数据抖动时，跳跃检测应触发
- ✅ 日志显示跳跃检测和锁定信息
- ✅ 返回正常区域后自动恢复

---

### 测试 6: 连续高速运动测试

**目的**：验证正常高速运动不会误触发

**步骤**：
1. 以接近但不超过阈值的速度（约 3-4 m/s）连续运动
2. 持续 60 秒以上

**预期结果**：
- ✅ 数据持续输出，无中断
- ✅ 日志无 JUMP 告警
- ✅ Filter 速度限幅生效（最大 2.0 m/s）

---

### 测试 7: Reset Service 测试

**目的**：验证重置原点功能

**步骤**：
```bash
# 移动 tracker 到新位置后
ros2 service call /reset_tracker_origin std_srvs/srv/Trigger "{}"
```

**预期结果**：
- ✅ Detector 和 Filter 状态被重置
- ✅ 日志显示重置信息
- ✅ 新的初始位姿被记录
- ✅ 输出立即从新原点继续

---

### 测试 8: 反复跳跃压力测试

**目的**：验证系统稳定性

**步骤**：
1. 快速甩动触发跳跃
2. 立即再次甩动（在恢复前再次触发）
3. 重复 10 次
4. 最后保持静止

**预期结果**：
- ✅ 系统保持锁定状态
- ✅ 不会输出异常数据
- ✅ 最终静止后能够恢复

---

### 测试 9: 恢复窗口内不稳定测试

**目的**：验证不会错误恢复

**步骤**：
1. 触发跳跃
2. 保持轻微震动（不完全静止），持续 2 秒
3. 观察是否保持锁定

**预期结果**：
- ✅ 日志持续显示方差值 > stability_threshold
- ✅ 系统保持锁定，不会错误恢复
- ✅ 只有真正稳定后才恢复

---

## 六、量化测试指标

| 指标 | 测试方法 | 期望值 | 实测值 |
|------|----------|--------|--------|
| 跳跃检测延迟 | 从跳跃发生到停止输出 | < 10ms | |
| 恢复时间 | 从稳定到恢复输出 | ~200ms | |
| 误触发率 | 正常运动被误判比例 | < 0.1% | |
| 滤波延迟 | 输入到输出延迟增加 | < 20ms | |
| 输出频率稳定性 | 正常情况频率波动 | 100Hz ± 2Hz | |

## 七、日志解读指南

### 7.1 正常状态日志
```
[INFO] [trackers_node]: [left] Pose normal, speed: 1.23 m/s
```

### 7.2 跳跃检测日志
```
[WARN] [trackers_node]: [left] [JUMP DETECTED] speed: 8.45 m/s (limit: 5.0 m/s), entering LOCKED state
```

### 7.3 锁定状态日志
```
[WARN] [trackers_node]: [left] [LOCKED] variance: 0.0023 (threshold: 0.0001), window: 5/20
```

### 7.4 恢复日志
```
[INFO] [trackers_node]: [left] [RECOVERED] variance: 0.00008, resuming output
```

### 7.5 滤波器日志
```
[DEBUG] [trackers_node]: [left] Filter: speed 2.34 -> 2.00 m/s (clamped)
```

## 八、故障排查

### 问题 1: 频繁误触发跳跃检测

**可能原因**：
- `jump_speed_limit` 设置过低
- Tracker 安装不稳定
- Base Station 覆盖不足

**解决方案**：
```yaml
detector:
  jump_speed_limit: 6.0  # 提高阈值
```

### 问题 2: 恢复时间过长

**可能原因**：
- `recovery_window_size` 设置过大
- `stability_threshold` 设置过低
- 环境震动

**解决方案**：
```yaml
detector:
  recovery_window_size: 15
  stability_threshold: 0.0004  # 放宽到 2cm 标准差
```

### 问题 3: 数据不够平滑

**可能原因**：
- `smoothing_alpha` 设置过大

**解决方案**：
```yaml
filter:
  smoothing_alpha: 0.2  # 降低使输出更平滑
```

### 问题 4: 响应延迟过大

**可能原因**：
- `smoothing_alpha` 设置过低

**解决方案**：
```yaml
filter:
  smoothing_alpha: 0.4  # 提高响应速度
```

## 九、测试检查清单

```
□ 环境准备
  □ SteamVR 正常运行
  □ Base Station 覆盖良好
  □ Tracker 电量充足
  □ 监控工具已启动

□ 测试场景
  □ 测试 1: 正常运动基准测试 - PASS / FAIL
  □ 测试 2: 跳跃触发测试 - PASS / FAIL
  □ 测试 3: 自动恢复测试 - PASS / FAIL
  □ 测试 4: 遮挡测试 - PASS / FAIL
  □ 测试 5: 边缘测试 - PASS / FAIL
  □ 测试 6: 高速运动测试 - PASS / FAIL
  □ 测试 7: Reset 服务测试 - PASS / FAIL
  □ 测试 8: 压力测试 - PASS / FAIL
  □ 测试 9: 不稳定恢复测试 - PASS / FAIL

□ 量化指标
  □ 跳跃检测延迟 < 10ms
  □ 恢复时间 ~200ms
  □ 误触发率 < 0.1%
  □ 输出频率 100Hz ± 2Hz

测试人员: ______________
测试日期: ______________
测试结果: PASS / FAIL
备注: ___________________
```

## 十、关键代码位置

| 功能 | 文件 | 行号参考 |
|------|------|----------|
| 跳跃检测逻辑 | `src/pipeline/detector.cpp` | `detectJump()` |
| 恢复判断逻辑 | `src/pipeline/detector.cpp` | `isWindowStable()` |
| 输出控制 | `src/trackers_node.cpp` | `processAndPublish()` |
| 滤波逻辑 | `src/pipeline/filter.cpp` | `apply()`, `clampVelocity()` |
| 参数配置 | `config/tracker_params.yaml` | - |
