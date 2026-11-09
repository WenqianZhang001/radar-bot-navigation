# Contributions and imported history

Wenqian Zhang implemented and physically deployed the TI AWR1843BOOST driver,
character-paced radar configuration, USB acquisition, measured mounting launch,
and browser point-cloud viewer. The canonical integration imports:

- `radar_navigation/driver/driver_node.py` and `radar_device.py` from
  `WenqianZhang001/radar-bot-navigation`, Jetson commit
  `9bee6926c19a846af50e3197726adbf625b8291b` (driver introduced in `528bccd`).
- `radar_viewer.py` including that commit's ROS-subscriber mode, which was not
  present on Wenqian's remote main when integration began.
- The physically verified radar profile from that checkout.

The repository preserves `9bee692` and its ancestors, including exact authorship,
as `docs/provenance/wenqian_9bee692.bundle`. The Jetson has no Git HTTPS push
credentials; publication uses the connected GitHub app, whose commit API cannot
import arbitrary original commit author/timestamp objects. The self-contained
bundle therefore preserves exact history without fabricating replacement SHAs:

```bash
git bundle verify docs/provenance/wenqian_9bee692.bundle
git fetch docs/provenance/wenqian_9bee692.bundle refs/heads/main:refs/remotes/wenqian-history/main
git log wenqian-history/main
```

The bundle is a small source-history archive, contains no raw recordings, and is
not a runtime dependency. Paths changed to fit one installable package;
startup cleanup, ambiguous-port rejection, default configuration, and signal
handling were adapted during integration. The hardware parser and wire schema
are retained. After the optional bundle fetch, `git log --all` exposes the
original authorship and changes.

Joseph E coordinates the canonical radar-navigation project. Its existing
history records the perception, replay, occupancy, control research, middleware
tests and physical experiment reports. Operator-supplied mounting/target surveys
and scene changes are explicitly attributed in the experiment metadata.

The original driver manifest carried `license: TODO`; this import does not
invent an upstream open-source license. This course project retains the
canonical repository's existing Proprietary designation pending a team decision.
