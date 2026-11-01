# ICD 2027 哨兵仿真启动

两个终端都从新的 Bash 会话开始，只加载本工作空间。无需设置 `ROS_DOMAIN_ID`。
本次更新包含 Gazebo 物理插件和三维真值桥接，需退出旧 Gazebo、导航和 RViz 后重新启动两套 launch。
当前工作空间已编译；仅重新发送 Nav2 Goal 不会让旧进程加载更新。

终端 1，启动 Gazebo（地图由 `external/icd2027sentry_gazebo_simulator/icd2027sentry_gazebo_simulator/config/gz_world.yaml` 选择，当前为 `rmul_2027`）：

```bash
cd /home/yyqff/sentry/icd2027_sentry_nav-main
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch icd2027sentry_gazebo_simulator icd2027sentry_bringup_sim.launch.py gz_args:=-r
```

终端 2，启动导航和 RViz：

```bash
cd /home/yyqff/sentry/icd2027_sentry_nav-main
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch icd2027sentry_nav_bringup icd2027sentry_navigation_simulation.launch.py world:=rmul_2027 slam:=False
```

## RMUL2027 自动过洞与定位来源

`world:=rmul_2027` 默认启用隧道行为树。RViz 的 `Nav2 Goal` 路径需要经过隧道时，
行为树负责洞外停车、关闭自旋并对齐、压低并等待反馈、必要时慢速居中，再沿通道穿洞，
完整车体离洞后恢复。停车、对齐和实际关节反馈确认仍是强制步骤，不会为提速跳过。
洞内取消导航时保持压低停车，不会自动抬升。静态地图开放的是经过三维几何检查的
通道及入口；侧墙和实时发现的低处障碍仍然参与避障。

RMUL2027 全局规划使用 `FootprintGridPlanner`：在整车物理碰撞包络的配置空间中执行 A*，
检查填充的车体区域和相邻路段扫掠，不再只用中心点或内切圆决定能否通过拐角。
普通跟随遇到拐角时会尝试更近的插值前瞻点和更低速度，并逐一检查实际运动的完整车体扫掠；
没有安全候选命令时仍停车，而不是放宽障碍阈值。

停车对齐区距顶板入口 `1.9 m`，并向场内横向偏移 `0.8 m`，避免在贴边界墙的窄通道里转身。
到开阔等待点前保持普通运动模式并允许重规划；自旋仍受完整车体守卫约束，
不在离隧道数米处提前锁住某一航向。到达等待点后才执行强制停车与轴向对齐。
对齐后先横移到中心线、再轴向进入；出洞控制也覆盖两侧窄引导段，不只覆盖顶板的 `0.35 m` 长度。
处于已居中、已对齐的引导段时，新目标可原地停车压低再通行，无需绕回外侧等待点。
顶板外的狭窄引导段允许高姿态车体沿通道轴线进出，但不在夹墙区域重新开启普通自旋；
若需改变航向，先安全离开引导段。目标横向偏离通道中心线不超过 `0.10 m` 时，
可投影到中心线执行，以避免末端横移碰墙；这仍在普通导航 `0.15 m` 的位置容差内。
若目标位于顶板下则保持低姿结束，完整车体已离开顶板才允许恢复高度。
隧道接近和穿越的末端采用线性减速，避免与距离 PID 叠加造成二次减速、长时间挪动。
普通及居中跟随先补密稀疏路径，防止近前瞻点落到身后的旧起点而反复前后运动。

最终速度守卫对**实际输出**（含行为树对齐和普通自旋）做完整车体扫掠检查：
普通导航仅在去除自旋后仍可安全平移时放行，且等待实际停转。
接近和穿洞控制在预测会越过目标点撞到远处边界时，只降低平移速度，保持必要的航向控制；
控制器与最终守卫均重新验证完整一秒的扫掠。没有安全速度、当前车体已碰撞或必要转动不安全时仍整车停车。
普通导航、隧道接近和轴向穿越共用 `1.8 m/s` 的速度上限，精细居中限制为 `0.2 m/s`；
实际速度还受剩余距离、障碍和反馈状态限制，并非全程以最高速度行驶。
RMUL2027 局部地图扩展为 `7×7 m`，为完整车体的一秒运动扫掠保留空间；
提速不缩短碰撞检查时域，也不缩小机器人碰撞包络。
局部地图、实际姿态或必要关节反馈过期时不会盲行。
穿洞中的升降、云台反馈或局部净空数据短暂失效时，行为树取消当前跟随并保持低姿停车；有效数据连续稳定恢复后
从当前位置重建并跟随剩余安全路段。持续失效超过 `5 s` 则任务失败，仍保持低姿；这不会放宽障碍判定。

当前 RMUL2027 默认 `use_sim_ground_truth:=True`，使用 Gazebo 真值作为一致的仿真定位源：

- `chassis_pose_gt` 提供物理 `WorldPose` 的完整三维位置与姿态。扣除已知出生坐标及
  URDF 的固定底盘安装偏移，生成唯一的 `odom → base_footprint` TF；
  `map → odom` 再补回出生坐标。升降位移只计算一次，不以关节行程替代真实高度。
  旧的平面 `chassis_odometry_gt` 接口保留，其他场地的原有链路不切换到三维真值模式。
  对外 TF 和导航里程计限制为 `50 Hz`，保留完整真值历史用于扫描时刻插值，避免千赫兹仿真
  里程计占满导航回调队列。
- 使用同一时刻的机器人关节 TF，生成以真实 `gimbal_yaw` 为子坐标系的 `odometry`。
- 同时发布以真实 `front_mid360`（含安装姿态）为子坐标系的 `lidar_odometry`，供两个地形
  分析节点滚动其局部窗口；它与导航虚拟底盘位姿不是同一消息。
- 原始 `livox/lidar` 按点云时间和雷达外参变换成 `registered_scan`，供地形分析和代价地图使用；
  `sensor_scan` 保留为雷达坐标系下的显示点云。
- 两路点云均先在同一时刻的真实 `chassis` 坐标系中过滤车体与轮组自身回波，避免自车
  四角落在圆形清除区域外，被地形分析累积成“障碍物”；车体包络外的环境障碍不受影响。
- 此模式关闭 Point-LIO、loam_interface、sensor_scan_generation 及雷达格式转换节点，避免重复的
  TF、里程计或点云发布源。`use_composition:=True/False` 都遵守这一选择。

这个真值模式用于当前 CAD 场地的仿真导航。它保留加速、自旋和升降时的真实车体高度、
横滚和俯仰，避免把运动中的地面点投影成临时障碍；并未单独验收坡地、台阶等场景的完整通行。
雷达 GPU 传感器在 `front_mid360` 链接内部的额外 `0.03 m` 偏移也纳入点云变换。
所有变换使用消息时间；没有对应时间的 TF 或真值时会丢弃该帧并告警，不以最新 TF 代替。
这项选择不用于实车定位精度验证；其他场地和实车启动仍沿用原有 LIO 链路。

可观察以下话题（默认机器人命名空间 `/red_standard_robot1`）：

| 话题 | 内容 |
|---|---|
| `tunnel_navigation_state` | 行为树阶段及隧道名称 |
| `tunnel_policy` | `[模式, 目标偏航, 速度限制]`，模式 0 正常、1 停车、2 对齐、3 穿洞 |
| `tunnel_guard_state` | `[map_x, map_y, chassis_yaw, 实际线速度, 实际角速度, 压低就绪]` |
| `tunnel_local_clearance_ready` | 实际车高满足净空，且局部点云代价地图已更新 |
| `joint_states` | 包含升降和云台关节的实际位置 |
| `chassis_lift_cmd` | 底盘升降目标位置 |

离线复核地图和真值点云变换：

```bash
python3 tools/prepare_rmul2027_tunnels.py --check
python3 tools/test_prepare_rmul2027_tunnels.py
python3 tools/test_gazebo_ground_truth.py
python3 tools/test_tunnel_navigation_observer.py
```

地图处理依赖 NumPy、Pillow 和 PyYAML；真值几何测试需要先加载上述 ROS 环境。
地图修改前后及通道中心线可查看 `tools/artifacts/rmul2027_tunnels/tunnel_review.png`。
在专用 `ROS_DOMAIN_ID=91` 仿真和导航启动后，可以执行
`python3 tools/test_tunnel_navigation.py --timeout 180` 记录完整自动穿洞结果；
增加 `--cancel-in-tunnel` 测试洞内取消。该工具只发送及取消自身的导航目标，不启动或重置仿真。
从洞内取消状态继续测试时，使用 `--resume-inside`，工具会要求初始位置确实在顶板下且已压低。
普通导航、拐角及两侧隧道之间的连续目标切换可用以下工具记录：

```bash
python3 tools/test_navigation_route.py --domain-id 91
```

该工具仅用于单独启动的 `ROS_DOMAIN_ID=91` 隔离仿真；只发送或取消自身的导航目标，
不重置 Gazebo、不清空代价地图、不改参数，也不直接发送升降或速度指令。
输出包含目标结果、实际穿洞速度和命令被安全守卫拦截的时间，可用于复核连续导航表现。
本次默认出生点七目标连续通行、两侧高速取消/恢复及 127 项回归记录见
[`validation_smooth/README.md`](tools/artifacts/rmul2027_tunnels/validation_smooth/README.md)。
历史 `validation/` 中的 `0.2 m/s`、74 项回归及八个穿洞用例属于此前低速基线。

## 底盘升降

ICD 轮组通过 `chassis_lift_joint` 与中间车体连接。`0.0 m` 为正常高度，
`0.055 m` 为车体完全压低。压低后最高碰撞点约为 `0.233 m`，低于
`rmul_2027` 隧道约 `0.250 m` 的净空。自动导航使用行为树控制；手动调试接口仍有效，
可在停车时执行以下指令压低：

```bash
ros2 topic pub --once /red_standard_robot1/chassis_lift_cmd std_msgs/msg/Float64 "{data: 0.055}"
```

离开隧道后恢复：

```bash
ros2 topic pub --once /red_standard_robot1/chassis_lift_cmd std_msgs/msg/Float64 "{data: 0.0}"
```

北极熊工作空间使用独立终端。两套仿真仍使用相同的机器人 ROS 命名空间与部分话题；不要同时启动。

## ICD 外观模型

Gazebo 使用 `icd_sentry_body.obj` 和 `icd_sentry_wheels.obj` 两组显示网格，二者由
`icd2027sentry_model/装配体2_彩色.3MF` 转换而来。车体网格连接 `chassis`，轮组网格连接
`wheel_carrier`，因此升降时中央车体会相对轮组运动。OBJ、MTL 和贴图需放在同一目录；
重新导出装配体后可分别用 `tools/convert_icd_colored_3mf.py --part-set body` 和
`--part-set wheels` 重新生成。

车轮滚动、转向效果以及云台运动仍由独立的仿真关节和简化碰撞体负责；CAD 网格只用于显示。
装甲板数字 7 是贴在原装甲板外表面的仿真几何体。

当前 `MecanumDrive2` 在车体上施加平面驱动力，轮子的简化圆柱没有逐个模拟麦轮滚子。
ICD 轮接触采用 `mu=mu2=0.01` 的低阻力近似，使小速度横移能完成入口居中；这不是用于
研究真实轮胎附着或打滑的高保真接触模型。碰撞体尺寸、隧道尺寸和所有净空保护保持不变。
ICD 模型的纵向速度伺服设为增益 `500`、力限幅 `200 N`，与原横向伺服一致，
减少低摩擦模型在精确停车时的纵向滑行；其他模型仍保留插件原有默认参数。
