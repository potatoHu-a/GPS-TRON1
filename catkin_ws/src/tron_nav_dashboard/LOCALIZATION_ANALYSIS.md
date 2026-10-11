# Current Localization Analysis and Phase 2 Design

## Current implementation

`/gps/fix` is the receiver's original `sensor_msgs/NavSatFix`. The global
converter validates it, builds an automatic origin from the median of 20
accepted samples, rejects origin outliers, rejects fixes whose reported
horizontal sigma exceeds 8 m, rejects position jumps above 3 m/s, converts
WGS84 to ENU, and applies an EMA with alpha 0.3. It has no position deadband.

`gps_fastlio_fusion` is an offset corrector, not an EKF or UKF. FAST-LIO
supplies relative position, yaw and velocity. GPS updates the `map ->
open_odom` X/Y offset with fixed gain 0.12 and a 0.5 m per-update clamp. GPS
covariance is used only for a hard rejection threshold and does not change the
gain. With filtered GPS enabled, the fusion consumes the filtered PoseStamped,
which contains no covariance.

When FAST-LIO speed is below 0.05 m/s, fusion replaces each instantaneous GPS
target with a rolling median after the window fills. It still applies the
fixed-gain correction, so stationary GNSS jitter can continue moving the
fused position. GPS never corrects yaw: `map_odom_yaw` starts at zero and is
not updated. No explicit transform aligns FAST-LIO startup yaw with ENU east
/ north. A systematic heading or southward tracking bias is therefore
possible when the two frames do not already agree.

RMC course is motion direction over ground, not body heading. It is useful
only while moving fast enough; this dashboard marks it invalid below 0.3 m/s.

## Recommended phase 2

1. Add a stationary gate driven by FAST-LIO linear and angular velocity. Hold
   GPS position correction while stationary, while continuing to collect a
   robust median for the next moving update.
2. Preserve NavSatFix covariance through ENU conversion. Compute an adaptive
   position measurement weight from GNSS covariance plus a configurable
   process covariance instead of fixed gain.
3. Reject innovations with a 2D Mahalanobis gate before correction. Report
   innovation, normalized innovation squared, accepted/rejected state, and
   effective gain.
4. Introduce a shadow EKF first: state `[x, y, vx, vy]` or an error-state over
   FAST-LIO odometry; FAST-LIO provides prediction increments and GNSS ENU
   provides absolute X/Y measurements with covariance. Compare its output to
   the current `/open_nav/odom` without controlling the robot, then migrate
   after bag replay and straight-line tests.
5. Calibrate yaw independently. Preferred source is dual-antenna GNSS HDT.
   Otherwise estimate a constant ENU-to-FAST-LIO yaw offset from a deliberate
   straight-line run using reliable GNSS course above a speed threshold. Never
   use stationary RMC course as heading.
