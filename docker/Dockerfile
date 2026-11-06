# Reproducible build and test environment.
#   docker build -f docker/Dockerfile -t pnc-stack .
#   docker run --rm -it pnc-stack
FROM ros:jazzy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      clang-format \
      clang-tidy \
    && rm -rf /var/lib/apt/lists/*

# Phase 1 adds the simulator here:
#   ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup

WORKDIR /ws
COPY src/ src/

RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && colcon test --packages-select pnc_core \
    && colcon test-result --verbose

RUN echo "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash" >> /root/.bashrc
CMD ["bash"]
