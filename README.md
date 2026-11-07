# radar-bot-navigation

Radar-based navigation for a mobile robot, using a TI AWR1843BOOST mmWave radar connected to a Jetson Orin Nano.

## What's here

| Folder | What it is |
| --- | --- |
| [`radar_pointcloud/`](radar_pointcloud/) | Live point cloud viewer: configures the radar over USB and shows its points in a web browser. Setup and usage are in its [README](radar_pointcloud/README.md). |

Each part of the project gets its own top-level folder with its own README.

## Get the code

```bash
git clone https://github.com/WenqianZhang001/radar-bot-navigation.git
```

On the lab Jetson, the repo is already at `~/radar-bot-navigation`.

`.gitignore` keeps radar recordings (`recordings/`, `data/`, `*.db3`, `*.mcap`) and logs out of git. GitHub rejects files over 100 MB, so share large recordings another way.
