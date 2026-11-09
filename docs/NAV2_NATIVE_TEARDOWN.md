# Jetson optional Nav2 native teardown issue

Observed during November 8 consolidation on aarch64/Humble, installed
`nav2_costmap_2d` 1.1.20. This server is **not part of the stationary system**.

The real StaticLayer test successfully marks occupied cells and propagates an
all-unknown source map to the configured unmarked local costmap. After lifecycle
deactivate/cleanup, however, the native server exits with SIGSEGV. The old test
ignored process exit status and reported success; the strengthened test now
reports this failure instead of hiding it.

GDB located the crash in `rclcpp::CallbackGroup::~CallbackGroup()`, called during
`NodeBase` / `rclcpp_lifecycle::LifecycleNode` destruction. These did not fix it:

1. Explicit lifecycle shutdown after deactivate and cleanup.
2. A small standalone C++ diagnostic that destroys executor/node before calling
   `rclcpp::shutdown()`; it still exited -11. This diagnostic is retained in
   Jetson artifacts, not used as a production workaround.

No installed ROS library was changed. A separate test-cleanup bug was fixed so
an assertion failure still closes the Python ROS context and does not cause
cascading `Context.init() must only be called once` errors in later tests.

Evidence: `/home/tejas/radar_integration_artifacts/consolidation_20261108T1900/`,
including `nav2_debug.log`, `nav2_lifetime_build.log`, `nav2_lifetime_test.log` and
full middleware runs. Reproduce with:

```bash
ROS_DOMAIN_ID=89 python3 -m unittest discover -s tests_ros -p test_nav2_costmap.py -v
```

Full middleware discovery includes this optional failure on the observed Jetson.
The required stationary suite is `test_cloud_input.py` plus `test_system.py`;
dry watchdog/localization regressions are separate and must also remain checked.
Do not describe the full Jetson middleware suite as all passing until native
teardown is resolved. Investigate matched ROS/Nav2 package versions and a minimal
callback-group reproduction before enabling a physical Nav2 control milestone.
