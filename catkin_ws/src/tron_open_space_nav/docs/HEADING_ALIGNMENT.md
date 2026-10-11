# 人工绝对航向标定

这是第一版 session 级航向标定。FAST-LIO 原始 `/Odometry` 不会被修改；
`gps_fastlio_fusion` 先用 `S=diag(1,-1,-1)` 将 TRON1 的 FRD
`camera_init/body1` 位姿统一转换为 ROS REP-103 FLU，再将标定 offset 用于
`/open_nav/odom` 和 TF。

## 流程

1. 保持 `dry_run=true`，启动 fusion 和 Dashboard。
2. Dashboard 显示“未标定”时点击“标定当前朝向”。
3. 在地图上点击机器人现实中正前方的远处参考点，建议距离至少 20 m。
4. 确认对话框中的标准化 FAST-LIO yaw、地图方向、offset、修正后全局 yaw 和参考距离。
5. 确认后调用 `/open_nav/heading/set_alignment`。

offset 使用 `wrap_to_pi(global_heading - fastlio_yaw)`。ROS/map 约定为
`+x=East`、`+y=North`、正 yaw 逆时针。标定时会重新计算
`map_odom_x/y = current_global_position - R(new_yaw) * normalized_fastlio_position`，
因此当前位置不会瞬间跳变；后续 FAST-LIO 位移按新的旋转方向进入地图。

## 接口

- `/open_nav/heading/status` (`tron_open_space_nav/HeadingStatus`)
- `/open_nav/heading/set_alignment` (`tron_open_space_nav/SetHeadingAlignment`)
- `/open_nav/heading/clear_alignment` (`std_srvs/Trigger`)

每次 fusion/FAST-LIO 新启动时默认 `UNALIGNED`。检测到原始 `/Odometry` 时间戳倒退、
序号大幅回退或原始 FAST-LIO 位姿跳变超过配置阈值时清除标定。短暂断流不会清除。

`HeadingStatus` 同时发布 raw 与 normalized 的 x/y/yaw，便于在 dry-run
人工旋转时确认：物理顺时针对应 normalized yaw 减小，逆时针对应增加。
