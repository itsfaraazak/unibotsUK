# Cross-build kit — how the Pi 5 aarch64 artifact is made

Produces `deploy/artifacts/unibots-ws-arm64.tar.gz` on an x86 laptop, **without Docker
and without touching the Pi**. This documents the method so the package can be rebuilt /
updated whenever the source changes.

```
deploy/cross-build/
├── cross-build.sh            # host driver — run this on the laptop
├── provision-and-build.sh    # runs inside the sandbox (don't run directly)
└── README.md                 # this file
```

## TL;DR — rebuild the package

```bash
# one command; ~15-30 min (emulated). Re-run any time src/ changes.
deploy/cross-build/cross-build.sh
# → deploy/artifacts/unibots-ws-arm64.tar.gz
```

`FRESH=1 deploy/cross-build/cross-build.sh` wipes the rootfs and re-bootstraps from
scratch. Optimization flags come from `deploy/build.env` (same file the native and Docker
builds use) — edit there, nothing else.

## How it works (the method)

There is no Pi and no Docker in the loop. The laptop pretends to be an aarch64 Ubuntu box:

1. **QEMU user-mode + binfmt.** The host kernel has `binfmt_misc` registered for
   `aarch64-linux` (on NixOS: `boot.binfmt.emulatedSystems = [ "aarch64-linux" ];`). Any
   aarch64 ELF then runs transparently through `qemu-aarch64` — so an aarch64 `gcc`,
   `cmake`, `colcon`, and the compiled nodes all "just run" on the x86 host.

2. **Ubuntu arm64 base rootfs.** `cross-build.sh` downloads the official Ubuntu 26.04
   `ubuntu-base` arm64 tarball and extracts it to `deploy/cross-build/.work/rootfs`. That
   gives a real aarch64 `/usr/bin/gcc-15`, dpkg, apt, etc.

3. **Dependency closure without maintainer scripts.** `provision-and-build.sh` adds the
   ROS apt repo as `[trusted=yes]` (skips gpg — gnupg's postinst crashes under emulation),
   `apt-get --download-only` pulls the whole closure, then **`dpkg-deb -x` extracts every
   `.deb`** into the rootfs. Extraction only — no postinst/preinst runs (those crash or
   hang under QEMU). This is why apt's normal `install` is avoided.

4. **Sandboxed build with bubblewrap.** `cross-build.sh` launches `bwrap` with the rootfs
   as `/`, `--clearenv` and a minimal `PATH=/usr/sbin:/usr/bin:/sbin:/bin`, and binds
   `/nix` + `/run/binfmt` so the qemu interpreter is reachable. Inside, it sources
   `/opt/ros/lyrical/setup.bash` and runs `deploy/build.sh` with the Pi 5 flags. Result is
   tarred to `unibots-ws-arm64.tar.gz`.

### Two emulation-only quirks (and why the scripts handle them)

- **Host-toolchain leak.** Without `--clearenv`, bwrap inherits the host's PATH and the
  build resolves the host x86 `gcc`/`cmake` instead of the rootfs aarch64 ones. Symptom:
  `cc1: error: bad value 'cortex-a76' for '-mtune='` (an x86 compiler rejecting an arm
  flag). Fix: `--clearenv` + minimal PATH. **Also** wipe `ws/build` first — a stale
  `CMakeCache.txt` pins `CMAKE_C_COMPILER=/nix/.../gcc` and survives across runs.
  `cross-build.sh` deletes `build/ install/ log/` every run.

- **Debian split NumPy not importable under QEMU.** `python3-numpy` on Debian/Ubuntu is
  laid out so `import numpy` fails inside the emulator, which breaks CMake's `FindPython3`
  NumPy probe (needed by `rosidl_generator_py` for `unibots_msgs`). `rosidl` only needs
  the *headers*, so `provision-and-build.sh` exports
  `EXTRA_CMAKE_ARGS=-DPython3_NumPy_INCLUDE_DIR=/usr/lib/aarch64-linux-gnu/python3-numpy/numpy/_core/include`
  which `deploy/build.sh` appends to its cmake args. A native Pi build leaves this unset.

## What's excluded — `unibots_perception`

`ncnn` is **not** an apt package on Ubuntu 26.04 arm64, so perception is skipped in the
tarball (and `build.sh`'s `SKIP` list in the rootfs copy). To get perception, build `ncnn`
from source **on the Pi** and then `colcon build --packages-select unibots_perception`, or
just run the stack `camera:=false` for a motion/BT/control bring-up. A native on-Pi build
(see below) has the same ncnn requirement — it is a real dependency, not an emulation gap.

## Updating the package

| You changed... | Do |
|---|---|
| C++/Python source in `unibots_ws/src` | re-run `deploy/cross-build/cross-build.sh` |
| optimization flags | edit `deploy/build.env`, re-run cross-build.sh |
| apt dependencies | edit `deploy/packages.apt`, re-run with `FRESH=1` |
| ROS distro / Ubuntu release | edit `ROS_DISTRO` + `UBUNTU_CODENAME` in `build.env`, `FRESH=1` |

---

## Prefer editing on the Pi? Build natively (recommended for match-day edits)

The emulated cross-build is for producing the tarball **in advance**. If you want to
**edit on the Pi over SSH**, put the source on the Pi and build natively there — it is
simpler, faster (real A76, no emulation), and builds perception too. See
[`../INSTRUCTIONS.md`](../INSTRUCTIONS.md) §B "Source on the Pi".
