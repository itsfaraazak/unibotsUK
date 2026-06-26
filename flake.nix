{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
    nixpkgs-unstable.url = "github:nixos/nixpkgs/nixpkgs-unstable";
  };
  outputs = { self, nix-ros-overlay, nixpkgs, nixpkgs-unstable }:
    let
      devShells = nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ nix-ros-overlay.overlays.default ];
          };
          pkgsUnstable = import nixpkgs-unstable {
            inherit system;
            config.allowUnfree = true;
          };
        in {
          devShells.default = pkgs.mkShell {
            name = "UnibotsUK Fresher Force Software";
            packages = with pkgs; [
              # Build tools
              colcon
              cmake
              ninja
              pkg-config
              # Libraries
              ncnn
              opencv

              pkgsUnstable.nono
              pkgsUnstable.claude-code
              nodejs
              # ... other non-ROS packages

              (with pkgs.rosPackages.lyrical; buildEnv {
                underlay = true;
                paths = [
                  ros-core
                  rclcpp
                  sensor-msgs
                  cv-bridge
                  ament-cmake
                  ament-cmake-core
                  ament-index-cpp

                  # Gazebo + ROS Integration packages
                  #ros-gz
                  #ros-gz-bridge
                  #ros-gz-sim
                  #ros-gz-image
                  # ... other ROS packages
                ];
              })
            ];
          };
        });
    in
    devShells // {
      nixosConfigurations.unibots-rpi = nixpkgs.lib.nixosSystem {
        modules = [
          # Cross-compile: build on x86_64, target aarch64 — no binfmt/sudo needed
          {
            nixpkgs.buildPlatform.system = "x86_64-linux";
            nixpkgs.hostPlatform.system = "aarch64-linux";
            nixpkgs.overlays = [ nix-ros-overlay.overlays.default ];
          }
          "${nixpkgs}/nixos/modules/installer/sd-card/sd-image-aarch64.nix"
          ./nixos/configuration.nix
        ];
      };
    };
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
