> Historical integration/design record. For the consolidated working system,
> use [STATIONARY_SYSTEM.md](STATIONARY_SYSTEM.md). Do not source the old driver
> workspace or follow the earlier parallel serial/TF launch procedure.

# Integrating the deployed radar driver

Update: the subscriber mode below is now implemented. Follow
[JETSON_INTEGRATION.md](JETSON_INTEGRATION.md) for deployment. The later Jetson
inventory found local commit `9bee692` implementing the ROS subscriber viewer,
resolving the concurrency question below. Preserve that unpublished commit.
The original source-inspection findings below describe GitHub `528bccd`.

## Repository inspection, 2026-11-08

Remote main revisions confirmed:
- `jelsayyid/radar-navigation`: `facbc1060a375824ca45116224e56ebc45aae1b6`.
- `WenqianZhang001/radar-bot-navigation`: `528bccd22feb47843556d396181c68218e829e2c`.

Use radar-navigation as the proposed canonical perception/navigation repository.
Retain Wenqian Zhang's `awr1843_radar` package as the acquisition component in a
shared colcon workspace; do not merge the two serial implementations blindly.
Confirm local Jetson changes before consolidating package files or updating its
checkouts. Hardware deployment is reported by the team, not verified here.

## Integration decision

Add a PointCloud2 subscription input mode to the existing perception node, with
serial/replay modes retained. The external mode must never start its serial worker.
Use `/radar/points` as input, preserve the message timestamp and `radar_link`, and
publish processed results under `/radar_front` for existing downstream consumers.

The input is already ROS axes: forward, left, up. The existing `Filter.update`
converts TI coordinates internally. An adapter must explicitly invert that
conversion at the Frame boundary, or introduce an explicitly ROS-coordinate core
entry point; otherwise points will rotate twice. Map `velocity` to `doppler`
without changing its sign. Preserve aligned `snr` and `noise` in `side_info_db`.
Define missing-quality handling explicitly and test malformed and empty clouds.

The deployed driver's parser supplies a frame number, but `read_data` discards it.
PointCloud2 alone cannot recover it. Extend acquisition metadata with the real
frame number and sensor identifier; never present a subscriber counter as a
hardware frame number. Recording should retain source clouds, metadata, and TF.

Keep Wenqian's measured transform exactly once:
`base_footprint -> radar_link`, translation `(0.036, -0.065, 0.185)` meters,
zero roll/pitch/yaw. Do not pass these values unchanged to the existing demo's
`base_link -> radar_front` transform. Verify the robot's base_link relationship
and measurement-time `odom -> radar_link` before accumulating moving-platform
observations. The existing core only supports planar extrinsics.

Use one serial owner per device. Both the checked-in viewer and new driver open
serial ports exclusively. Their simultaneous operation remains unexplained until
local processes and changes are inspected. A future viewer should subscribe to
the acquisition output rather than open the same USB stream.

## Next action on the Jetson

Run `bash scripts/inspect_jetson.sh` from an agent connected to the Jetson and
retain its output outside the checkout. This bounded read-only inventory reports
repository status, serial identities, process names, nodes, cloud schema/QoS, and
TF. It neither starts acquisition nor commands motion. Run it in the same ROS
shell/domain as the deployed driver. Timeouts or missing topics are observations,
not proof of hardware failure. Inspect relevant local diffs with the operator;
do not overwrite uncommitted work. The script has been shell-syntax checked in
cloud, but its ROS commands have not been executed on the Jetson.

After inventory, implement/test the subscriber mode against actual schema/QoS,
then build both packages on Jetson. With the robot stationary, record surveyed
reflectors and demonstrate correct occupied cells, expiry to unknown, timestamp
freshness, and frame rates. Only then advance to supervised motion checks in
JETSON_DEMO.md. No physical navigation result is established by this inspection.

Cloud limitation: no configured VPN/TCP route to the Jetson and no local ROS
installation. A Jetson-connected agent or configured cloud VPN/SSH access is
required for the next physical milestone. Do not send SSH credentials in chat.
