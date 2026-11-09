# Radar-facing forward and native TurtleBot coordinates

Our canonical `base_footprint` +x points out of the radar antenna face; +y is
left viewed from behind the radar, +z up. The mount survey remains
(0.150,0.060,0.190) m with nominal zero radar rotation in that frame.

Do not infer physical axes from the labels of a ROS motor channel. November 8
supported-wheel observations exposed a mismatch between our radar-facing front
and the unmodified TurtleBot3 command/odometry convention. Forward positive native
commands rolled both tire tops away from an observer facing the antenna. Turning
still looked left; yaw sign alone could not establish forward alignment.

## Measured channel assignment

Scheduled isolation experiment, 2026-11-08 Eastern time:

| Pulse | Command v,w | Measured native left,right surface velocity | Operator observation facing antenna |
| --- | --- | --- | --- |
| A, 21:22:11 | +0.02 m/s, +0.14 rad/s | 0,+0.038777 m/s | Observer-right / radar-left wheel top away |
| B, 21:22:16 | +0.02 m/s, -0.14 rad/s | +0.039568,0 m/s | Observer-left / radar-right wheel top away |

Thus canonical wheel L = -native R; canonical wheel R = -native L. Consequently:

```text
v_native = -v_canonical       yaw_rate_native = yaw_rate_canonical
x_canonical = -x_native      y_canonical = -y_native
heading_canonical = heading_native  (both odom frames start at heading zero)
```

This is an observed actuator mapping, not a claim about an unseen chassis FRONT
arrow, firmware fault, swapped wiring or motor EEPROM configuration. No firmware,
IDs or drive-mode registers were changed. The QC sticker and gray component are
recorded visual references, not axis evidence.

## Implementation and TF

Stock turtlebot3_node 2.3.6 still owns OpenCR and integrates native wheel odometry.
The independent watchdog converts canonical commands only at its private driver
output. Its limits, leases and decision/status values remain canonical;
`native_linear/native_angular` expose the actual driver command in status.

```text
odom
  -- fixed yaw pi --> native_odom
       -- stock dynamic wheel odometry --> native_base_footprint
            -- fixed yaw pi --> base_footprint
                 -- fixed z .010 --> base_link
                 -- one surveyed mount --> radar_link
            -- stock URDF native_* robot links (robot_state_publisher)
```

The first half-turn defines world axes; the second defines body axes. They are
not two radar mount corrections. Their initial orientation cancels to canonical
heading zero. The stock URDF uses `frame_prefix=native_` and its original native
joints, preserving native robot geometry rather than pretending its left/right
labels became radar-facing labels. Only one dynamic TF publisher defines the
odometry edge, and only the existing `radar_tf` publishes the radar mount.

`base_frame_adapter` republishes `/odom` from `/base/native_odom`, retaining the
original header timestamp. It rotates world/body vectors and covariance cross
terms; it does not integrate or fabricate odometry. For general quaternions the
half-turn conjugation negates qx/qy and preserves qz/qw. It publishes canonical
`/joint_states` with swapped/negated wheel channels. Native joint/IMU data have
explicit native frames on `/base/native_joint_states` and `/base/native_imu`;
raw driver messages remain under `/base/*_raw`. `/sensor_state` motor encoder
left/right labels remain native channels; use canonical `/joint_states` or
`/odom` for radar-facing kinematics. The guard uses only its torque/voltage.
No canonical IMU alignment is
claimed. Default wheel-only odometry remains the physically tested mode.

The adapter's static transforms supply the sole canonical base_link joint and
coordinate bridges. If the adapter exits, launch stops the base; missing adapter
odometry also closes the watchdog's freshness gate. There is no parallel direct
odom->base_footprint TF or static fake wheel odometry. Perception continues to
look up the full chain at radar receipt time, with bounded interpolation retry.

## Validation and limits

Offline tests cover wheel-command consistency, curved-path geometry and covariance
cross terms. Dry real-hardware tests compare matched native/canonical samples and
measurement-time TF against canonical `/odom`, and check mount ownership. Physical
corrected direction and stop tests are recorded in PROGRESS.md and compact results.
Earlier pre-adapter motor runs remain evidence of native stop behavior, not proof
of physical canonical-forward correctness. Stationary radar experiments retain
valid radar-facing coordinates because they did not depend on native base odom.

Supported-wheel integration is not floor-motion accuracy: chassis remains still
while encoder odometry moves. Restart with fresh odom/map before any separately
approved floor experiment. Floor wheel geometry, traction, drift, sensor timing
and stopping-distance validation remain outstanding.
