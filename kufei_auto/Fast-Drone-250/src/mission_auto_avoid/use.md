# mission_auto_avoid.launch 使用说明

本文档对应文件：

- `src/mission_auto_avoid/launch/mission_auto_avoid.launch`

当前版本已经按实机链路调整过默认接线：

- 默认不启用 `airsim_bridge`
- 默认点云话题是 `/world_cloud`
- 默认里程计话题是 `/mavros/local_position/odom`
- 控制入口默认使用 `apm_auto.launch`
- 默认不启动 `xyz_input_node`
- 默认 `mission_source=fcu`

## 1. 作用概述

这条 launch 负责把下面几部分接起来：

- `ego_planner`
  以外部目标模式运行，订阅 `/ego_input_target`，输出 `/position_cmd`
- `apm_auto`
  负责 `LOITER` 和 `AUTO/GUIDED` 两态控制，并在 `GUIDED` 时按条件接管 `/position_cmd`
- `mission_waypoint_uploader.py`
  在 `mission_source:=file` 时，把 `mission_waypoints.yaml` 上传成飞控 `AUTO` mission
- `auto_avoid_manager.py`
  检测前方障碍，决定什么时候 `AUTO -> GUIDED -> AUTO`

整条链路的核心话题关系是：

- `auto_avoid_manager.py` 发布 `/ego_input_target`
- `ego_planner` 订阅 `/ego_input_target`，输出 `/position_cmd`
- `auto_avoid_manager.py` 发布 `/mission_auto_avoid/avoidance_active`
- `apm_auto` 订阅 `/mission_auto_avoid/avoidance_active` 和 `/position_cmd`
- `apm_auto` 最终向 `mavros/setpoint_raw/local` 发控制点

## 2. 默认实机前提

默认假设外部已经有这些话题：

- `/mavros/local_position/odom`
  无人机里程计
- `/world_cloud`
  世界系障碍点云
- `/mavros/state`
  飞控当前模式和连接状态
- `/mavros/local_position/pose`
  当前位置
- `/mavros/rc/in`
  遥控器输入
- `/mavros/mission/waypoints`
  飞控任务航点
- `/mavros/global_position/gp_origin`
  FCU mission 转本地 ENU 所需的原点
- `/mavros/home_position/home`
  仅在返航模式也要走这条避障链路时需要
  manager 会用它给 planner 提供返航避障目标

如果你的实机不是这些话题，需要在 launch 参数里覆盖。

## 3. 启动内容

`mission_auto_avoid.launch` 会启动或包含：

- 可选的 `airsim_bridge/lidar_to_world_enu.launch`
  只有 `use_airsim_bridge:=true` 时才启动
- `planner_external_target.launch`
  外部目标版 planner
- `px4/apm_auto.launch`
  默认只启动 `apm_auto_node`
  只有显式传 `enable_xyz_input_node:=true` 时才会启动 `xyz_input_node`
- `mission_waypoint_uploader.py`
  仅 `mission_source:=file` 时启动
- `auto_avoid_manager.py`

## 4. 运行链路

默认 `mission_source=fcu` 时，运行顺序是：

1. `auto_avoid_manager.py` 等待 `/mavros/mission/waypoints` 和 `/mavros/global_position/gp_origin`
2. manager 从 FCU mission 中解析当前 `current_seq` 对应的目标点和航段
3. 正常情况下飞控保持原自动模式飞行
   - 普通任务航点时通常是 `AUTO`
   - 如果返航是由遥控器或地面站切出来的，也可能是 `RTL/RTN`
4. manager 持续检查当前航段前方和局部重接线方向上的障碍
5. 若前方障碍触发避障，manager 发布新的 `/ego_input_target`
6. planner 生成 `/position_cmd`
7. manager 确认这次 `/position_cmd` 足够新、轨迹已就绪、首跳不过大后，请求飞控切到 `GUIDED`
8. `apm_auto` 只有在：
   - 遥控器处于 `AUTO/GUIDED` 档
   - 飞控实际模式是 `GUIDED`
   - `/mission_auto_avoid/avoidance_active=true`
   时才接管 `/position_cmd`
9. 障碍消失、轨迹重接原航段并满足退出条件后，manager 再请求飞控回到“进入避障前的原模式”
   - 进入前是 `AUTO`，就回 `AUTO`
   - 进入前是 `RTL/RTN`，就回对应返航模式
10. 若进入避障前是返航模式，manager 给 planner 的临时目标不是硬编码原点，而是 `/mavros/home_position/home` 的本地 home 点
    - 水平位置取 `home.x/home.y`
    - 高度保持当前 `z`
    - 完整返航剖面仍由飞控在退出 `GUIDED` 后继续执行

## 5. 主要参数

下面只列最常用、最关键的参数。

### 5.1 基础环境

- `drone_id`
  默认 `0`
  多机时用于 planner 命名空间

- `obj_num`
  默认 `10`
  动态障碍物数量相关参数，主要传给 planner

- `map_size_x`
- `map_size_y`
- `map_size_z`
  默认 `400 / 400 / 7`
  地图尺寸

### 5.2 传感器与环境输入

- `odom_topic`
  默认 `/mavros/local_position/odom`
  planner 和 manager 使用的主里程计

- `lidar_world_topic`
  默认 `/world_cloud`
  世界系点云

- `lidar_world_odom_topic`
  默认跟 `odom_topic` 相同
  仅在 `use_airsim_bridge:=true` 时作为桥接输出 odom 话题

- `use_airsim_bridge`
  默认 `false`
  只有 AirSim 仿真时才打开

- `input_cloud_topic`
  默认 `/airsim_node/Copter/lidar/Lidar1`
  这是 AirSim 原始点云入口，只在 `use_airsim_bridge:=true` 时有意义
  实机默认直接吃 `/world_cloud`，不会用到这个话题

### 5.3 Planner 参数

- `max_vel`
  默认 `1.0`

- `max_acc`
  默认 `2.0`

- `planning_horizon`
  默认 `6`

- `use_distinctive_trajs`
  默认 `false`
  表示关闭“一次重规划中同时尝试多条不同绕障候选轨迹”
  实机默认保持关闭，以降低计算量和时延

- `planning_enabled_on_start`
  默认 `false`
  当前链路里 planner 开关由 `auto_avoid_manager.py` 调用 `set_planning_enabled` 联动控制
  因此不建议上电就让 planner 持续工作

- `planning_service`
  默认 `/drone_0_ego_planner_node/set_planning_enabled`
  manager 通过它开关 planner

### 5.4 模式切换与控制

- `mission_source`
  默认 `fcu`
  可选：
  - `file`
    使用本地 `waypoint_file` 或 launch 里的 `mission/pointN_*`
  - `fcu`
    直接读取飞控 mission，适合遥控器上传航点后的 `AUTO/GUIDED` 切换链路

- `guided_mode`
  默认 `GUIDED`

- `auto_mode`
  默认 `AUTO`

- `return_mode_names`
  默认 `RTL,RTN,AUTO.RTL,AUTO.RTN`
  这些模式会被 manager 识别成“返航类自动模式”
  如果在这些模式下触发避障，manager 会临时切 `GUIDED`，退出后恢复原模式

- `loiter_mode`
  默认 `LOITER`

- `avoidance_flag_topic`
  默认 `/mission_auto_avoid/avoidance_active`
  manager 通过它告诉 `apm_auto`：“现在允许避障接管”

- `planner_cmd_topic`
  默认 `/position_cmd`
  planner 输出轨迹采样点

- `set_auto_on_start`
  默认 `true`
  启动后是否自动请求切到 `AUTO`
  当 `mission_source:=fcu` 时，manager 内部会按 `false` 处理，避免在还没拿到 FCU mission 前主动切模

- `auto_start_delay`
  默认 `2.0`
  自动请求 `AUTO` 前的等待时间

- `avoidance_takeover_max_setpoint_jump`
  默认继承 `takeover_max_cmd_distance`
  用来限制 `GUIDED` 接管时首个 setpoint 跳变

- `enable_xyz_input_node`
  定义在 `px4/apm_auto.launch`
  默认 `false`
  当前 `mission_auto_avoid` 链路下建议保持关闭，避免和 `auto_avoid_manager.py` 同时向 `/ego_input_target` 发布目标

### 5.5 任务与航点

- `waypoint_file`
  默认 `$(find mission_auto_avoid)/config/mission_waypoints.yaml`

- `mission_ready_param`
  默认 `/mission_auto_avoid/mission_ready`

- `mission_start_wp_seq`
  默认 `0`

- `mission_set_current_after_upload`
  默认 `false`

- `mission_prepend_home_placeholder`
  默认 `true`

- `mission_waypoint_hold_time`
  默认 `3.0`

- `fcu_waypoints_topic`
  默认 `/mavros/mission/waypoints`
  仅 `mission_source:=fcu` 时使用

- `fcu_origin_topic`
  默认 `/mavros/global_position/gp_origin`
  仅 `mission_source:=fcu` 时使用

- `fcu_home_topic`
  默认 `/mavros/home_position/home`
  仅 `mission_source:=fcu` 时使用
  当实际飞控模式属于 `return_mode_names` 时，manager 用它给 planner 提供返航避障目标

- `fcu_set_current_service`
  默认 `/mavros/mission/set_current`
  仅 `mission_source:=fcu` 时使用

- `sync_fcu_current_on_auto_resume`
  默认 `false`
  仅 `mission_source:=fcu` 时使用
  如果开启，manager 在 `GUIDED -> AUTO` 前会尝试把飞控 `current_seq` 推进到当前机体附近的后续 mission 点

- `fcu_set_current_dist_threshold`
  默认继承 `waypoint_reached_radius`
  仅 `mission_source:=fcu` 时使用
  用于判断“当前是否已经足够接近后续 mission 点，可以显式 `set_current`”

### 5.6 避障阈值

- `route_obstacle_distance_threshold`
- `route_clear_distance_threshold`
- `route_lookahead_distance`
- `route_min_forward_distance`

这些控制沿原始 mission 航线前看的障碍判断。

- `local_obstacle_distance_threshold`
- `local_clear_distance_threshold`
- `local_lookahead_distance`
- `local_min_forward_distance`

这些控制局部避障重接线方向上的障碍判断。

- `route_rejoin_tolerance`
- `route_rejoin_enter_tolerance`
- `rejoin_forward_distance`

这些控制“是否已经回到原 mission 航段附近，可以准备退回 AUTO”。

### 5.7 GUIDED 接管/退出稳定性

- `guided_hold_speed_threshold`
  切到 `GUIDED` 后，要等机体速度降下来再放 planner 目标

- `planner_cmd_timeout`
  多久收不到新 `/position_cmd` 就认为 planner 输出不新鲜

- `planner_valid_hold_time`
  planner 输出满足条件后，要持续多久才算稳定

- `planner_motion_recent_window`
  用于判定 planner 近期是否真的在输出“有运动意义”的轨迹

- `planner_cmd_min_speed`
- `planner_cmd_min_acc`
- `planner_cmd_min_pos_error`

这些控制 manager 对 `/position_cmd` 的“有效轨迹”判定阈值。

- `require_new_planner_traj`
  是否要求这次避障必须看到新的 `trajectory_id`

- `takeover_max_cmd_distance`
  manager 判定第一帧 planner cmd 是否离当前 odom 太远

- `avoidance_min_duration`
  最短避障持续时间

- `clear_hold_time`
  前方清空后还要保持多久才允许退出 `GUIDED`

- `guided_exit_debounce`
  最终 `GUIDED -> AUTO` 的防抖时间

## 6. 关键话题

### 6.1 外部必须提供

- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/local_position/odom`
- `/mavros/rc/in`
- `/world_cloud`

默认 `mission_source=fcu` 时还需要：

- `/mavros/mission/waypoints`
- `/mavros/global_position/gp_origin`

若希望 `RTL/RTN` 也走同一条避障链路，还需要：

- `/mavros/home_position/home`

### 6.2 本 launch 内部关键话题

- `/ego_input_target`
  `auto_avoid_manager.py` 发布，planner 订阅

- `/position_cmd`
  planner 输出，`auto_avoid_manager.py` 和 `apm_auto` 都会订阅

- `/mission_auto_avoid/avoidance_active`
  `auto_avoid_manager.py` 发布，`apm_auto` 订阅

- `/mode_select_flag`
  `apm_auto` 发布的内部模式标志
  当前版本只表示：
  - `0`: LOITER 档
  - `1`: AUTO/GUIDED 档

- `/mavros/setpoint_raw/local`
  `apm_auto` 最终输出给飞控

### 6.3 调试话题

`auto_avoid_manager.py` 还会发布一组调试量：

- `~closest_distance`
- `~route_closest_distance`
- `~local_closest_distance`
- `~route_distance`
- `~obstacle_ahead`
- `~planner_cmd_valid`
- `~auto_resume_ready`

## 7. 默认模式逻辑

当前控制链路配合 `apm_auto` 的行为是：

- 遥控器切到 `LOITER` 档：
  - 控制器停发外部 setpoint
  - 飞控保持 `LOITER`
  - planner 是否工作由 manager 的规划器开关决定，不靠 `LOITER` 直接杀 planner

- 遥控器切到 `AUTO/GUIDED` 档：
  - 如果飞控当前就是 `AUTO`，则优先按 `AUTO` mission 飞
  - 如果飞控当前已经被外部切到 `RTL/RTN` 等其它自动模式，`apm_auto` 不会强制把它拉回 `AUTO`
  - 只有当 `auto_avoid_manager.py` 检测到障碍并发布 `avoidance_active=true`
  - 且 manager 成功请求飞控实际切到 `GUIDED`
  - 且 `/position_cmd` 满足接管条件
  - `apm_auto` 才会允许 `/position_cmd` 接管

- 若在 `RTL/RTN` 等返航模式下触发避障：
  - manager 会记录进入避障前的原模式
  - 避障期间临时切到 `GUIDED`
  - planner 输入目标取 FCU home 点的本地坐标
  - 避障结束后恢复进入前的返航模式

## 8. 常用启动方式

### 8.1 实机默认启动

```bash
roslaunch mission_auto_avoid mission_auto_avoid.launch
```

这要求你已经有：

- `/mavros/local_position/odom`
- `/mavros/local_position/pose`
- `/mavros/state`
- `/mavros/rc/in`
- `/world_cloud`
- `/mavros/mission/waypoints`
- `/mavros/global_position/gp_origin`

### 8.2 指定实机点云/odom 话题

```bash
roslaunch mission_auto_avoid mission_auto_avoid.launch \
  odom_topic:=/odom_in_world \
  lidar_world_topic:=/world_cloud \
  lidar_world_odom_topic:=/odom_in_world
```

### 8.3 文件航点模式

```bash
roslaunch mission_auto_avoid mission_auto_avoid.launch \
  mission_source:=file \
  waypoint_file:=$(rospack find mission_auto_avoid)/config/mission_waypoints.yaml
```

这个模式会启动 `mission_waypoint_uploader.py`，把本地 YAML 上传到飞控 mission。

### 8.4 AirSim 模式

```bash
roslaunch mission_auto_avoid mission_auto_avoid.launch \
  use_airsim_bridge:=true \
  odom_topic:=/mavros/local_position/odom \
  input_cloud_topic:=/airsim_node/Copter/lidar/Lidar1 \
  lidar_world_topic:=/lidar_in_world \
  lidar_world_odom_topic:=/lidar_in_world_odom
```

只有在你的 ROS 环境里确实能找到 `airsim_bridge` 包时，才能启用这条路径。

## 9. 当前已知注意事项

- `auto_avoid_manager.py` 会调用 `planning_service`
  默认是 `/drone_0_ego_planner_node/set_planning_enabled`
  如果当前 planner 代码还没有这个 service，运行时会持续警告，但 launch 本身仍可起来

- 当前 `apm_auto` 是两态逻辑：
  - `LOITER`
  - `AUTO/GUIDED`
  不再区分旧版内部的 `AUTO_HOLD / AUTO_COMMAND / GUIDED` 三段小状态

- 当前 `apm_auto` 在 `mode_select_flag=1` 时，如果飞控实际模式不是 `AUTO` 也不是 `GUIDED`，不会再强制切回 `AUTO`
  这保证了遥控器或地面站已经切出来的 `RTL/RTN` 等自动模式不会被中间层打断

- 当前 launch 默认已经偏向实机，不再默认走 AirSim 桥接

- `input_cloud_topic` 不是 planner 直接订阅的话题
  它只是 AirSim 原始点云入口
  真正喂给 planner 和 manager 的仍然是 `lidar_world_topic`

- `planner_external_target.launch` 现在的默认地图和实机 planner 链路对齐为：
  - `map_size_x=400`
  - `map_size_y=400`
  - `map_size_z=7`
  - `use_distinctive_trajs=false`

- 返航模式下，`auto_avoid_manager.py` 不会重建飞控完整返航策略
  它只是在避障期间把 planner 目标设为 `fcu_home_topic` 提供的 home 本地位置，并保持当前高度
  真正的返航航迹、返航高度、降落策略仍由飞控在退出 `GUIDED` 后继续执行
