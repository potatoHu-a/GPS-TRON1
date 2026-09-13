# 室外定位精度诊断指南

## 定位链

```
/gps/fix (WGS84)
  -> gps_global_converter -> /global_pose, /open_nav/gps_raw_pose, /open_nav/gps_filtered_pose
  -> gps_fastlio_fusion + /Odometry -> /open_nav/odom
  -> open_nav_path_publisher -> /open_nav/path
```

## 20m 定距测试步骤

1. 室外开阔地，启动 `gps_global_nav.launch`（含 `localization_diagnostics`）
2. 站在起点：
   ```bash
   rosservice call /open_nav/localization/reset_distance "{}"
   ```
3. 沿直线走 **20m**（卷尺/测步）
4. 设置实测距离：
   ```bash
   rosservice call /open_nav/localization/set_measured "{distance_m: 20.0}"
   ```
5. 查看日志 summary 或：
   ```bash
   rostopic echo /open_nav/localization_status -n 1
   ```

## 如何判断问题在哪一层

| 现象 | 可能原因 |
|------|----------|
| GPS raw ≈ 实测，GlobalPose 明显偏小 | ENU 转换 bug（运行 `test_geodetic_utils.py`） |
| GPS raw 本身偏小 | 手机 GPS 多路径/遮挡 |
| GlobalPose 正常，FAST-LIO 偏小 | FAST-LIO 尺度/漂移 |
| FAST-LIO 正常，OpenNavOdom 偏小 | 融合 gain 过大/过强平滑 |
| filtered/gps_raw ratio < 0.85 | EMA 过强（降低 `gps_ema_alpha`） |
| open_nav/gps < 0.85 | 融合层问题 |

## 回滚

恢复 `config/gps_global.yaml` 与 `config/global_fusion.yaml` 旧参数：
- `origin_samples: 1`（等价首帧 origin）
- `enable_gps_filter: false`
- `use_filtered_gps_pose: false`
- `gps_correction_gain: 0.15`（旧 alpha）

或 git checkout 对应文件。
