> Historical integration/design record. For the consolidated working system,
> use [STATIONARY_SYSTEM.md](STATIONARY_SYSTEM.md). Do not source the old driver
> workspace or follow the earlier parallel serial/TF launch procedure.

# Deploy and validate external-driver occupancy

## Inventory baseline (reported November 8, 2026)

Jetson: Ubuntu 22.04.5/aarch64, Humble, domain 30, Python 3.10.
`/home/tejas/radar-bot-navigation` was clean at local commit
`9bee6926c19a846af50e3197726adbf625b8291b`; preserve this commit and ignored
workspace outputs. It adds the ROS subscriber viewer and is newer than GitHub
`528bccd`. No radar, viewer, TF, or odometry was running. Only OpenCR enumerated;
XDS110 was absent. These observations do not establish a hardware fault.

## Instructions for the Jetson-connected agent

Deployment and stationary radar testing are authorized. Preserve teammate work.
Do not reset or overwrite the existing repository or start motor/control nodes.
Reinspect local changes, processes, and serial enumeration before acting. Stop
if another process owns the radar; coordinate rather than killing it.

1. Preserve Wenqian's local commit. Do not replace her checkout with GitHub's
   older revision. Source `/opt/ros/humble/setup.bash` and her existing
   `ros2_ws/install/setup.bash`; use `ROS_DOMAIN_ID=30` for physical tests.
2. Clone `https://github.com/jelsayyid/radar-navigation.git` into a new
   `~/radar_integration_ws/src/radar_navigation` path if absent. If already
   present, inspect status before a fast-forward update. Verify the requested
   integration commit is present. Build this overlay from `~/radar_integration_ws`
   using `colcon build --symlink-install --packages-select radar_navigation`.
   Source its `install/setup.bash` after Wenqian's overlay. Keep a transcript and
   outputs outside both repositories.
3. Run `python3 -m unittest discover -s src/radar_navigation/tests -v`.
   Run the new middleware test on an isolated domain:
   `ROS_DOMAIN_ID=89 python3 -m unittest discover -s src/radar_navigation/tests_ros -p test_cloud_input.py -v`.
   This exercises a synthetic best-effort publisher, measured mount, occupancy,
   stale health and map expiry without hardware. Report it separately from real
   radar results. If any check fails, diagnose before physical validation.
4. Require XDS110 if00/if03 enumeration. If absent, ask the operator to connect
   USB and power, then recheck. Do not select OpenCR as a radar interface.
5. Start `ros2 launch awr1843_radar radar.launch.py` using the preserved package.
   This is the sole serial owner and sole radar mount publisher. Confirm the
   installed mount is `(0.036, -0.065, 0.185)` relative to base_footprint.
6. In the same sourced environment start
   `ros2 launch radar_navigation cloud.launch.py`.
   It subscribes to `/radar/points` in `radar_link`, outputs under `/radar_front`,
   and starts no serial driver, static TF publisher, or controller. The default
   fixed frame is base_footprint **for a stationary robot only**. Before any
   movement, supply valid measurement-time odometry and use `fixed_frame:=odom`.
   Never invent a static odom transform for a moving robot.
7. Optionally launch the existing viewer with `--ros` from Wenqian's checkout.
   Never use its default serial mode concurrently. Preserve its local changes.
8. Record `/radar/points`, `/radar_front/points_raw`, `/radar_front/points`,
   `/radar_front/metadata`, `/radar_front/status`, `/radar_front/occupancy`,
   `/tf_static`, and `/tf` with rosbag2 to a timestamped directory outside Git.
   Run `ros2 run radar_navigation radar_probe --seconds 30 --output <report.json>`.
   Retain source cloud quality fields and TF in the bag; processed clouds carry
   x/y/z/doppler while quality remains in metadata. Hardware frame gaps are null:
   the driver does not publish its hardware counter. Subscriber sequence is not
   a hardware frame number.
9. With operator assistance, place reflectors at measured positions, verify
   forward/left/up signs and occupied cells, remove them and observe expiry to
   unknown. Stop only the acquisition process started for this test and verify
   stale status within roughly 0.5 s after the last accepted frame and map expiry
   after 1.5 s plus the 0.2 s map timer. Report actual measured timings; do not
   infer physical obstacle removal merely from a synthetic test. Empty frames
   are valid; a positive min_snr rejects unknown/NaN quality.
10. Return commit IDs, build/test outcomes, bag/report paths, actual source and
    processed rates, TF checks, surveyed obstacle errors, expiry/stale timing,
    and remaining blockers. Distinguish source inspection from live measurement.
    Do not perform autonomous motion in this deployment task.

No cloud result establishes Jetson compatibility or physical radar performance.
