# Unibots Pi 5 — Step-by-Step Deploy Instructions

Follow top to bottom. Three stages: **prep (once)** → **cross-build (before the
event)** → **match day (on the Pi)**. A fallback (Ansible) and an edit-on-the-day
procedure are at the end.

All paths produce the **same** Cortex-A76-optimized binaries — flags live only in
`deploy/build.env`.

## Pick a path

| Path | On the Pi you need | Pi compiles? | Edit on Pi? | Perception (ncnn)? |
|---|---|---|---|---|
| **Tarball** (§A — fastest boot) | apt ROS only — **no Docker, no Nix** | no | no | no¹ |
| **Source on Pi** (§B — best for edits) | apt ROS + git | yes (native, fast) | **yes, via SSH** | yes¹ |
| **Docker** (§1) | Docker | no | rebuild in container | yes (in image) |
| **Ansible** (fallback) | nothing (provisioned from laptop) | yes (~15-25 min) | ssh + rebuild | yes |

¹ `unibots_perception` needs `ncnn`, which is **not** an apt package on Ubuntu 26.04
arm64 — it must be built from source on the Pi. The tarball ships the other 6 packages;
run perception-off (`camera:=false`) for a motion/BT/control/servo bring-up, or build
ncnn + perception on the Pi separately.

---

## A. Prebuilt tarball — no Docker, no Nix on the Pi (recommended)

The artifact `deploy/artifacts/unibots-ws-arm64.tar.gz` is a ready-to-run aarch64
`install/` tree (Cortex-A76 `-O3 -flto`). The Pi only needs the ROS 2 `lyrical` apt
packages; untar and run — zero compiling on the Pi.

```bash
# 1) ship it
scp deploy/artifacts/unibots-ws-arm64.tar.gz pi@robot:~/

# 2) on the Pi — ROS lyrical from apt (one time; see deploy/packages.apt for full list)
sudo apt-get update
sudo apt-get install -y ros-lyrical-ros-base ros-lyrical-rosidl-default-runtime \
    python3-numpy libeigen3-dev      # + remaining runtime deps in deploy/packages.apt

# 3) drop the workspace + run
mkdir -p ~/unibots_ws && tar -C ~/unibots_ws -xzf ~/unibots-ws-arm64.tar.gz
source /opt/ros/lyrical/setup.bash
source ~/unibots_ws/install/setup.bash
ros2 launch unibots_bt match.launch.py home_zone:=north hardware:=true camera:=false
```

Then start the match (§5). To **rebuild / update** this tarball from the laptop, see
`deploy/cross-build/README.md` — one command: `deploy/cross-build/cross-build.sh`.

---
USING THIS
## B. Source on the Pi — edit + rebuild over SSH (match-day edits)

Put the repo on the Pi and build **natively** there. No Docker, no Nix, no emulation —
real Cortex-A76, fast incremental rebuilds, and it builds `unibots_perception` too (once
ncnn is present). This is the path to use if you might need to change code at the event.

```bash
# 1) one time on the Pi — ROS + build tools from apt
sudo apt-get update
xargs -a <(sed 's/@ROS_DISTRO@/lyrical/g; s/#.*//' deploy/packages.apt) \
    sudo apt-get install -y          # full dep list lives in deploy/packages.apt
#    (skip libncnn-dev — not in apt; see "ncnn" below)

# 2) get the source onto the Pi (git clone, or rsync from the laptop)
git clone <your-repo-url> ~/unibotsUK         # or: rsync -a ./ pi@robot:~/unibotsUK/
cd ~/unibotsUK

# 3) native optimized build (~15-25 min first time, incremental after)
WS=$PWD/unibots_ws bash deploy/build.sh
source unibots_ws/install/setup.bash
ros2 launch unibots_bt match.launch.py home_zone:=north hardware:=true
```

**Edit on match day over SSH:**
```bash
ssh pi@robot
cd ~/unibotsUK
nano unibots_ws/src/unibots_bt/bt/game_tree.xml      # or any source
WS=$PWD/unibots_ws bash deploy/build.sh              # incremental rebuild (only the changed pkg)
sudo systemctl restart unibots                       # if running under systemd; else relaunch
```

> The BT tree is pure XML (`unibots_ws/src/unibots_bt/bt/game_tree.xml`) and most tuning is
> live ROS params (`TUNING.md`) — many match-day tweaks need **no rebuild at all**.

**ncnn / perception on the Pi:** `libncnn-dev` is not in apt. Build it from source once,
then the build picks up `unibots_perception`:
```bash
sudo apt-get install -y build-essential cmake libprotobuf-dev protobuf-compiler libomp-dev
git clone https://github.com/Tencent/ncnn ~/ncnn && cd ~/ncnn
cmake -B build -DCMAKE_BUILD_TYPE=Release -DNCNN_VULKAN=OFF && cmake --build build -j4
sudo cmake --install build
# then remove unibots_perception from SKIP in deploy/build.sh and rebuild
```

Auto-start on boot is the same `unibots.service` idea as §4 but pointing `run.sh` at the
native workspace instead of Docker.

---

## 0. What you need

**Dev laptop (x86):** Docker + buildx, this repo checked out.
**Robot:** Raspberry Pi 5, Ubuntu 26.04 (aarch64), Docker installed, camera +
motor/servo wired, on the same network as your laptop.

```bash
# On the Pi, one time:
sudo apt-get update && sudo apt-get install -y docker.io
sudo usermod -aG docker "$USER"     # log out/in after this
```

---

## 1. Cross-build the image IN ADVANCE (on the laptop)

Do this the night before — it is slow (emulated arm64).

```bash
cd <repo root>
./deploy/docker/buildx.sh
```

Output: `deploy/artifacts/unibots-pi5-arm64.tar`.

> Reproducible build: edit `deploy/build.env` first if you want to change the ROS
> distro or optimization flags. Pin `ARG BASE_DIGEST=@sha256:...` in
> `deploy/docker/Dockerfile` to freeze the Ubuntu base.

---

## 2. Ship it to the robot (laptop → Pi)

```bash
scp deploy/artifacts/unibots-pi5-arm64.tar  unibots@unibots-pi5.local:~/
# copy the compose + runtime files too (if the repo isn't already on the Pi):
rsync -a deploy  unibots@unibots-pi5.local:~/unibotsUK/
```

On the Pi, load the image (no compiling):

```bash
ssh unibots@unibots-pi5.local
docker load -i ~/unibots-pi5-arm64.tar      # → image unibots:pi5
```

---

## 3. Configure the match (on the Pi)

Edit `~/unibotsUK/deploy/runtime.env` — set the wall the judges give you:

```ini
HOME_ZONE=north        # north | east | south | west
CONTROLLER=mpc         # mpc | apf
HARDWARE=true
CAMERA=true
LOCALIZATION=true
USE_SIM_TIME=false
ROS_DOMAIN_ID=42
```

---

## 4. Run + auto-start on boot (on the Pi)

```bash
cd ~/unibotsUK
docker compose -f deploy/docker/docker-compose.yml --env-file deploy/runtime.env up -d
docker compose -f deploy/docker/docker-compose.yml logs -f      # watch it come up
```

Start on power-up automatically:

```bash
sudo cp deploy/docker/unibots.service /etc/systemd/system/
sudoedit /etc/systemd/system/unibots.service     # set REPO= / WorkingDirectory to ~/unibotsUK
sudo systemctl daemon-reload
sudo systemctl enable --now unibots
```

---

## 5. Start the match

Robot stays **idle** until the start signal (rulebook §1.9). The start topic is
**latched** — the durability flag is mandatory or the message is dropped:

```bash
ros2 topic pub --qos-durability transient_local /match/start \
    std_msgs/msg/Bool '{data: true}' --once
```

Watch it think:

```bash
ros2 topic echo /game/state      # IDLE → SEARCH → HUNT → CAPTURE → ... → PARKED
ros2 topic echo /game/target
```

---

## FALLBACK — Ansible (native, no Docker)

If Docker misbehaves, provision the Pi bare-metal from the laptop:

```bash
ansible-galaxy collection install -r deploy/ansible/requirements.yml
# edit deploy/ansible/inventory.ini  (Pi host + user)
cd deploy/ansible
ansible-playbook site.yml            # installs ROS, builds optimized, starts service
```

This compiles ON the Pi (~15–25 min). Same `build.env` flags. Then do step 5.

---

## MATCH-DAY EDIT (should not happen)

You changed code and must rebuild fast:

**Docker (src is bind-mounted):**
```bash
docker compose -f deploy/docker/docker-compose.yml exec unibots \
    bash -lc 'WS=/opt/unibots/unibots_ws bash /opt/unibots/deploy/build.sh'
docker compose -f deploy/docker/docker-compose.yml restart
```

**Ansible / native:**
```bash
ssh unibots@robot
cd ~/unibotsUK/unibots_ws && nano src/...
WS=$PWD bash ../deploy/build.sh
sudo systemctl restart unibots
```

---

## TROUBLESHOOTING

| Symptom | Fix |
|---|---|
| Robot won't start after publish | Missing `--qos-durability transient_local` on `/match/start`. |
| `permission denied` GPIO/I2C/camera | Pi user not in `gpio/i2c/video` groups, or compose not `privileged`. |
| Image won't `load` | Re-pull the tarball; check `arch` is arm64 (`docker image inspect unibots:pi5`). |
| Link error during build | Set `ENABLE_LTO=0` in `deploy/build.env`, rebuild. |
| Tracker / pose drifting | Confirm `ENABLE_FAST_MATH=0` (default) — fast-math breaks Kalman/EKF. |
| Perception node missing / `ncnn` not found | Tarball excludes `unibots_perception` (ncnn not in apt). Build ncnn from source on the Pi then `colcon build --packages-select unibots_perception`, or run `camera:=false`. |
| ROS pkgs not found (apt) | ROS distro name wrong for Ubuntu 26.04 → fix `ROS_DISTRO` in `build.env`. |
| Nodes can't see each other | Same `ROS_DOMAIN_ID` everywhere; host networking on. |

Full reference: `deploy/README.md`. Tuning: `TUNING.md`. Architecture: `CLAUDE.md`.
