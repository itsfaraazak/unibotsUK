{ pkgs, lib, config, ... }:

let
  workspace = import ./workspace.nix { inherit pkgs; };

  ros = pkgs.rosPackages.lyrical;

  rosEnv = ros.buildEnv {
    underlay = true;
    paths = with ros; [
      ros-core rclcpp sensor-msgs cv-bridge
      nav-msgs geometry-msgs std-msgs visualization-msgs
      robot-localization apriltag-ros
    ];
  };

  launchScript = pkgs.writeShellScript "unibots-launch" ''
    source ${rosEnv}/setup.bash
    source ${workspace}/setup.bash
    exec ros2 launch unibots_game match_day.launch.py use_sim_time:=false
  '';

in {
  # RPi4 hardware
  boot.loader.grub.enable = false;
  boot.loader.generic-extlinux-compatible.enable = true;

  networking.hostName = "unibots";
  networking.useDHCP = true;

  services.openssh.enable = true;
  services.openssh.settings.PermitRootLogin = "yes";

  users.users.robot = {
    isNormalUser = true;
    password = "unibots";
    extraGroups = [ "video" "dialout" "tty" ];
  };

  # ROS2 domain isolation
  environment.variables.ROS_DOMAIN_ID = "42";
  environment.variables.ROS_LOCALHOST_ONLY = "1";

  systemd.services.unibots = {
    description = "Unibots Robot";
    wantedBy = [ "multi-user.target" ];
    after = [ "network.target" "local-fs.target" ];
    serviceConfig = {
      ExecStart = launchScript;
      Restart = "always";
      RestartSec = "3s";
      User = "robot";
      # Camera access
      SupplementaryGroups = "video";
    };
  };

  environment.systemPackages = [
    workspace
    rosEnv
    pkgs.htop
  ];

  sdImage.compressImage = true;
  system.stateVersion = "24.11";
}
