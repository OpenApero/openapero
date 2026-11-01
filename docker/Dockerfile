# syntax=docker/dockerfile:1
#
# Open Apero on ROS Noetic (Ubuntu 20.04). Build from the repository root, with submodules
# initialized:
#
#   docker build -f docker/Dockerfile -t open_apero .
#
# OR-Tools is downloaded from ORTOOLS_URL; `--build-context ortools=<dir>` replaces it with a local
# directory holding ortools.tar.gz (BuildKit then uses it in place of the stage of the same name).
# The voxblox fork comes with the context, as the deps/voxblox submodule. See docker/README.md.

ARG ROS_IMAGE=ros:noetic-ros-base-focal

# ---------------------------------------------------------------------------------------------
# OR-Tools v9.11 prebuilt, as /ortools.tar.gz (open_apero_core links ortools/lib/libortools.so).
FROM ${ROS_IMAGE} AS ortools_download
ARG ORTOOLS_URL=https://github.com/google/or-tools/releases/download/v9.11/or-tools_amd64_ubuntu-20.04_cpp_v9.11.4210.tar.gz
RUN apt-get update && apt-get install -y --no-install-recommends wget ca-certificates \
 && mkdir /out && wget -q -O /out/ortools.tar.gz "${ORTOOLS_URL}"

FROM scratch AS ortools
COPY --from=ortools_download /out/ /

# ---------------------------------------------------------------------------------------------
FROM ${ROS_IMAGE} AS workspace
ARG WS=/root/catkin_ws
ARG BUILD_TYPE=RelWithDebInfo
ARG JOBS=
ENV DEBIAN_FRONTEND=noninteractive WS=${WS}
SHELL ["/bin/bash", "-c"]

# docs/installation.md, Prerequisites and Workspace and Dependencies, with apt recommends off here and for rosdep below. rviz comes in anyway:
# open_apero_ros, open_apero_graph_benchmark and voxblox_rviz_plugin depend on it, so this image
# can show RViz over X11 (docker/run.sh --gui); no desktop-full image is needed.
RUN echo 'APT::Install-Recommends "false";' > /etc/apt/apt.conf.d/99-no-recommends \
 && apt-get update && apt-get install -y \
      python3-catkin-tools libassimp-dev git wget autoconf libtool \
      ros-noetic-ompl ros-noetic-jsk-rviz-plugins \
 && rm -rf /var/lib/apt/lists/*

# docs/installation.md, Workspace and Dependencies: the sources. The submodules (deps/voxblox) must be initialized in the context.
COPY . ${WS}/src/open_apero

# docs/installation.md, rosdep. Noetic is end-of-life, so rosdep resolves nothing without --include-eol-distros.
RUN source /opt/ros/noetic/setup.bash \
 && apt-get update \
 && rosdep update --include-eol-distros \
 && rosdep install --from-paths ${WS}/src --ignore-src -y --rosdistro noetic \
 && rm -rf /var/lib/apt/lists/*

# OR-Tools setup: include/ and lib/ into open_apero_core/ortools/.
RUN --mount=type=bind,from=ortools,target=/mnt/ortools \
    mkdir -p /tmp/ortools ${WS}/src/open_apero/open_apero_core/ortools \
 && tar -xzf /mnt/ortools/ortools.tar.gz -C /tmp/ortools \
 && cp -r /tmp/ortools/or-tools_*/include /tmp/ortools/or-tools_*/lib ${WS}/src/open_apero/open_apero_core/ortools/ \
 && rm -rf /tmp/ortools

# Build. SLIM=true (default) drops build/ and logs/ and strips the devel-space binaries, 7.2 -> 5.0 GB:
# everything still runs, and the next `catkin build` of a package reconfigures it from scratch.
# SLIM=false keeps build/ for fast incremental rebuilds inside the container.
ARG SLIM=true
WORKDIR ${WS}
RUN source /opt/ros/noetic/setup.bash \
 && catkin init \
 && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
 && catkin build --no-status --summarize -j${JOBS:-$(nproc)} \
 && if [ "${SLIM}" = true ]; then \
      rm -rf build logs \
      && find devel/.private src/open_apero/open_apero_core/ortools/lib -type f \( -name '*.so*' -o -perm -u+x \) \
           -exec sh -c 'strip --strip-debug "$1" 2>/dev/null || true' _ {} \; ; \
    fi

# docker/run.sh runs as the host user: let any user reach the workspace below /root. Run outputs go to
# ~/open_apero_data, which run.sh mounts from the host, so nothing in the workspace needs to be writable.
RUN chmod 755 /root

# RViz (docker/run.sh --gui) otherwise opens a modal end-of-life dialog and ignores Ctrl+C. Dockerfile.habitat
# inherits it.
ENV DISABLE_ROS1_EOL_WARNINGS=1

COPY docker/entrypoint.sh /entrypoint.sh
RUN echo "source ${WS}/devel/setup.bash" >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
