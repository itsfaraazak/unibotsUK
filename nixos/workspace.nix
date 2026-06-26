{ pkgs, ... }:

let
  ros = pkgs.rosPackages.lyrical;

  # Build the entire colcon workspace as a single Nix derivation.
  # When nixpkgs.buildPlatform = x86_64 and hostPlatform = aarch64,
  # stdenv uses the aarch64 cross-compiler automatically.
  workspace = pkgs.stdenv.mkDerivation {
    name = "unibots-workspace";
    src = ../unibots_ws/src;

    nativeBuildInputs = [
      pkgs.colcon
      pkgs.cmake
      pkgs.ninja
      pkgs.pkg-config
      pkgs.python3
      pkgs.python3Packages.setuptools
      (ros.buildEnv {
        underlay = true;
        paths = with ros; [
          ros-core
          rclcpp
          sensor-msgs
          cv-bridge
          ament-cmake
          ament-cmake-core
          ament-index-cpp
          nav-msgs
          geometry-msgs
          std-msgs
          visualization-msgs
          robot-localization
          apriltag-ros
        ];
      })
    ];

    buildInputs = [
      pkgs.ncnn
      pkgs.opencv
      pkgs.eigen
    ];

    buildPhase = ''
      export HOME=$(mktemp -d)
      export COLCON_HOME=$HOME/.colcon

      colcon build \
        --packages-skip unibots_description cvxpygen unibots_robot_gazebo unibots_sim \
        --build-base $TMPDIR/colcon-build \
        --install-base $out \
        --cmake-args \
          -DCMAKE_BUILD_TYPE=Release \
          -DEigen3_DIR=${pkgs.eigen}/share/eigen3/cmake
    '';

    installPhase = "true";
  };

in
workspace
