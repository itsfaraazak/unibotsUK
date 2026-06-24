{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
    nixpkgs-unstable.url = "github:nixos/nixpkgs/nixpkgs-unstable";
  };
  outputs = { self, nix-ros-overlay, nixpkgs, nixpkgs-unstable }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
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
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
