# Unibots Pi 5 Deployment

Reproducible, infrastructure-as-code deploy for the competition robot:
**Raspberry Pi 5 / Ubuntu 26.04 (aarch64)**, ROS 2 `lyrical`, every C++ node
compiled with **max Cortex-A76 optimization**.

Two interchangeable paths — pick one; if it fails on match day, fall back to the
other. Both consume the **same** build core, so the binary is identical:

| | When to use | Boot | Match-day edit |
|---|---|---|---|
| **Docker** | Default. Immutable, deterministic boot. | `systemd → docker compose up` | bind-mounted `src/` → `colcon build` in the container |
| **Ansible** | Fallback / bare-metal. No container layer. | `systemd → run.sh` | `ssh`, edit, `colcon build`, `systemctl restart` |

---

## Single source of truth

Everything tunable lives in three files — edit these, nothing else:

| File | Controls |
|---|---|
| `deploy/build.env` | ROS distro, Pi 5 CPU target, `-O3`/LTO/extra flags (the optimization) |
| `deploy/packages.apt` | apt build + runtime dependencies (`@ROS_DISTRO@` auto-substituted) |
| `deploy/runtime.env` | match-day runtime: `HOME_ZONE`, `CONTROLLER`, `HARDWARE`, ... |

`deploy/build.sh` (build) and `deploy/run.sh` (launch) are shared by both paths.

### Optimization flags (`build.env`)

```
-O3 -mcpu=cortex-a76+crypto -mtune=cortex-a76
-funroll-loops -fomit-frame-pointer -fno-plt -fno-semantic-interposition -pipe
-flto=auto -ffat-lto-objects          # LTO (ENABLE_LTO=1)
```
`-ffast-math` is **off by default** — it degrades the Kalman tracker and EKF.
`ENABLE_LTO=0` disables LTO if a link ever fails under time pressure.

---

## Path A (proven) — Prebuilt tarball, no Docker / no Nix on the Pi

The cross-built `deploy/artifacts/unibots-ws-arm64.tar.gz` is a raw aarch64 `install/`
tree (Cortex-A76 `-O3 -flto`). The Pi needs only apt ROS `lyrical` — untar and run, no
container, no compile. This is the path used for hardware bring-up. Step-by-step in
`deploy/INSTRUCTIONS.md` §A; build provenance + reproduce in `deploy/artifacts/README.md`.
Excludes `unibots_perception` (ncnn not in apt — build from source on the Pi, or run
`camera:=false`).

---

## Path A0 — Cross-compile in advance (Docker image variant)

Build the **aarch64 / Cortex-A76-optimized** image on your x86 laptop *before*
the event, save a tarball, and the Pi 5 never compiles — boot is just `load` +
`up`. Uses Docker buildx + QEMU; the `build.sh` inside still emits real
`-mcpu=cortex-a76` binaries (QEMU emulates the A76 instructions).

```bash
# On the x86 dev machine (needs docker + buildx):
./deploy/docker/buildx.sh
# → deploy/artifacts/unibots-pi5-arm64.tar   (slow: emulated build)

# Copy to the robot and load — no build on the Pi:
scp deploy/artifacts/unibots-pi5-arm64.tar unibots@robot:~/
ssh unibots@robot 'docker load -i ~/unibots-pi5-arm64.tar'

# On the Pi 5:
docker compose -f deploy/docker/docker-compose.yml --env-file deploy/runtime.env up -d
```

Or push to a registry instead of a tarball:
```bash
PUSH=1 IMAGE=ghcr.io/you/unibots:pi5 ./deploy/docker/buildx.sh
```

Reproducible: pin `ARG BASE_DIGEST=@sha256:...` in the Dockerfile, commit it, and
the same tarball is reproducible from the same `src/` + digest.

---

## Path A — Docker, build on the Pi

(Use when you don't have a prebuilt tarball from Path A0.)

```bash
# On the Pi 5, from the repo root:
docker compose -f deploy/docker/docker-compose.yml build

# Run the match stack:
docker compose -f deploy/docker/docker-compose.yml --env-file deploy/runtime.env up -d
docker compose -f deploy/docker/docker-compose.yml logs -f
```

Boot on power-up:
```bash
sudo cp deploy/docker/unibots.service /etc/systemd/system/
sudoedit /etc/systemd/system/unibots.service   # set REPO= / WorkingDirectory
sudo systemctl daemon-reload && sudo systemctl enable --now unibots
```

**Match-day edit (should not happen):**
```bash
# src/ is bind-mounted — rebuild incrementally inside the running container:
docker compose -f deploy/docker/docker-compose.yml exec unibots \
    bash -lc 'WS=/opt/unibots/unibots_ws bash /opt/unibots/deploy/build.sh'
docker compose -f deploy/docker/docker-compose.yml restart
```

Reproducibility: pin the base by setting `ARG BASE_DIGEST=@sha256:...` in the
Dockerfile and commit it. Same digest + same `src/` ⇒ same image.

---

## Path B — Ansible (native, fallback)

```bash
ansible-galaxy collection install -r deploy/ansible/requirements.yml
# edit deploy/ansible/inventory.ini  (Pi host + user)
ansible-playbook deploy/ansible/site.yml          # cwd: deploy/ansible (ansible.cfg)
```

Runs `system → ros → build → service`. Installs the systemd unit and starts it.

**Selective re-runs:**
```bash
ansible-playbook deploy/ansible/site.yml --tags build      # rebuild only
ansible-playbook deploy/ansible/site.yml --tags service    # push runtime.env only
```

**Match-day edit (should not happen):**
```bash
ssh unibots@robot
cd ~/unibotsUK/unibots_ws && nano src/...        # edit
WS=$PWD bash ../deploy/build.sh                  # optimized incremental build
sudo systemctl restart unibots
```

---

## Start a match

Both paths leave the robot **idle** until the physical start signal (rulebook
§1.9). The BT gates on a *latched* `/match/start`:

```bash
ros2 topic pub --qos-durability transient_local /match/start \
    std_msgs/msg/Bool '{data: true}' --once
```

Monitor: `ros2 topic echo /game/state` · `ros2 topic echo /game/target`

---

## Notes / assumptions

- Assumes the ROS 2 `lyrical` apt repo is published for Ubuntu 26.04. If the
  distro name differs, change `ROS_DISTRO` in `build.env` (one place).
- The build skips `unibots_description`, `unibots_sim`, `unibots_behavior_tree`
  (not part of the competition runtime).
- GPIO/I2C/V4L2 need device access: Docker runs `privileged` with `/dev` mapped;
  Ansible adds the user to `gpio/i2c/video/dialout` + installs udev rules.
- `restart` policies only recover a crashed process — they never re-assert
  `/match/start`, so the robot cannot auto-drive after a reset.
