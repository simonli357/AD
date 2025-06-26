{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    nixpkgs-unstable.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  outputs = { self, nix-ros-overlay, nixpkgs, nixpkgs-unstable }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        unstable = import nixpkgs-unstable {
          inherit system;
        };
        pkgs = import nixpkgs {
          inherit system;
          config = {
            allowUnfree = true;
            cudaSupport = true;
            permittedInsecurePackages = [
              "freeimage-3.18.0-unstable-2024-04-18"
            ];
          };
          overlays = [
            nix-ros-overlay.overlays.default
            (final: prev: {
              tbb = prev.tbb_2021_11;
              tensorrt = prev.cudaPackages.tensorrt_8_6;
              opencv = prev.opencv.overrideAttrs (old: {
                cmakeFlags = old.cmakeFlags ++ [
                  (prev.lib.cmakeBool "WITH_CUDA" true)
                  (prev.lib.cmakeBool "CUDA_FAST_MATH" true)
                  (prev.lib.cmakeFeature "CUDA_ARCH_BIN" "7.5")
                  (prev.lib.cmakeFeature "CUDA_NVCC_FLAGS" "-allow-unsupported-compiler")
                ];
              });
              acados = prev.callPackage ./acados.nix {};
            })
          ];
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "ROS";

          NIX_CFLAGS_COMPILE = pkgs.lib.concatStringsSep " " [
            "-Wno-error=implicit-function-declaration"
            "-Wno-error=incompatible-pointer-types"
            "-Wno-error=int-conversion"
          ];
          NIX_CXXFLAGS_COMPILE = pkgs.lib.concatStringsSep " " [
            "-Wno-error=implicit-function-declaration"
            "-Wno-error=incompatible-pointer-types"
            "-Wno-error=int-conversion"
          ];

          shellHook = ''
            export SHELL=/run/current-system/sw/bin/zsh
            export GAZEBO_MODEL_PATH="/home/admin/Containers/ros/Simulator/src/models_pkg:$GAZEBO_MODEL_PATH"
            export ROS_PACKAGE_PATH="/home/admin/Containers/ros/Simulator/src:$ROS_PACKAGE_PATH"
            export LD_LIBRARY_PATH="/home/admin/Containers/ros/AD/devel/lib:$LD_LIBRARY_PATH"
            source /home/admin/Containers/ros/AD/devel/setup.sh
          '';

          packages = [
            # C++ Tools
            pkgs.stdenv.cc
            pkgs.cmake
            pkgs.gnumake
            pkgs.clang-tools
            pkgs.llvmPackages.openmp
            pkgs.gdb
            pkgs.valgrind
            # Graphics drivers
            pkgs.mesa
            pkgs.libglvnd
            pkgs.vulkan-loader
            pkgs.vulkan-validation-layers
            pkgs.qt5.full
            pkgs.linuxPackages.nvidia_x11
            pkgs.libGLU
            pkgs.libGL
            pkgs.glm
            pkgs.libxkbcommon
            pkgs.wayland
            pkgs.wayland-protocols
            # Dependencies
            pkgs.uwsm
            pkgs.ninja
            pkgs.mbed-cli
            pkgs.mercurial
            pkgs.tensorrt
            pkgs.udev
            pkgs.ncurses
            pkgs.librealsense-gui
            pkgs.tbb
            pkgs.nlohmann_json
            pkgs.eigen
            pkgs.opencv
            pkgs.acados
            pkgs.ncnn
            pkgs.hwloc
            pkgs.gazebo
            pkgs.ignition.cmake2
            pkgs.xwayland
            pkgs.freetype
            pkgs.gnuplot
            pkgs.ranger
            pkgs.tcpdump
            (pkgs.python3.withPackages (python-pkgs: with python-pkgs; [
              requests psutil python-lsp-server setuptools distutils pyqt5 pyglm numpy pyyaml pandas pyopengl pyopengl-accelerate cryptography twisted pillow scipy networkx matplotlib freetype-py
            ]))
            (with pkgs.rosPackages.noetic; buildEnv {
              paths = [
                # base
                ackermann-msgs
                urdf
                camera-info-manager
                ros-core
                rospy
                roscpp
                cv-bridge
                tf2
                tf
                image-transport
                geometry-msgs
                tf2-geometry-msgs
                gazebo-msgs
                gazebo-ros
                robot-localization
                # slam
                gmapping
              ];
            })
          ];

          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
            "${pkgs.tensorrt}/lib"
            "${pkgs.opencv}/lib"
            "${pkgs.acados}/lib"
            "${pkgs.mesa.drivers}/lib"
            "${pkgs.libGL}/lib"
          ];

          TRT_LIBPATH = "${pkgs.tensorrt}/lib";
          TBB_DIR = "${pkgs.tbb}/lib/cmake/TBB";
          nlohmann_json_DIR = "${pkgs.nlohmann_json}/share/cmake/nlohmann_json";
          Eigen3_DIR = "${pkgs.eigen}/share/eigen3/cmake";
          OpenCV_DIR = "${pkgs.opencv}/lib/cmake/opencv4";
          OpenCV_INCLUDE_DIRS = "${pkgs.opencv}/include/opencv4";
          acados_DIR = "${pkgs.acados}/cmake";
          ncnn_DIR = "${pkgs.ncnn}/lib/cmake/ncnn";
          GDK_BACKEND = "x11";
          QT_QPA_PLATFORM = "xcb";
          PYOPENGL_PLATFORM="x11";
          SDL_VIDEODRIVER = "x11";
          GTK_MODULES="canberra-gtk-module";
          QT_OPENGL="desktop";
          LIBGL_DRIVERS_PATH="${pkgs.mesa.drivers}/lib/dri";
        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
