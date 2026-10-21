# koide3 direct_visual_lidar_calibration (stock, unmodified) + Xvfb + initial_guess_from_points: a separate small
# program (initial_guess_from_points.cpp) that calls koide's own PoseEstimation on the 2D-3D correspondences stored in
# calib.json, i.e. the headless equivalent of the "Estimate" button of koide's initial_guess_manual.
#   docker build -t koide-vlcal:release calibration/scripts/koide
FROM koide3/direct_visual_lidar_calibration:humble@sha256:f7363cc3deadc2419f3527a7b9ec7bdabaa3031fd8e1b9c6cc46666ae1301bbb
RUN apt-get update && apt-get install -y --no-install-recommends xvfb xauth && rm -rf /var/lib/apt/lists/*
COPY initial_guess_from_points.cpp /tmp/
RUN D=/root/ros2_ws/src/direct_visual_lidar_calibration && cp /tmp/initial_guess_from_points.cpp $D/src/ && \
    printf '\nadd_executable(initial_guess_from_points src/initial_guess_from_points.cpp)\ntarget_link_libraries(initial_guess_from_points direct_visual_lidar_calibration)\ninstall(TARGETS initial_guess_from_points DESTINATION lib/${PROJECT_NAME})\n' >> $D/CMakeLists.txt && \
    cd /root/ros2_ws && . /opt/ros/humble/setup.sh && \
    colcon build --packages-select direct_visual_lidar_calibration --cmake-args -DCMAKE_BUILD_TYPE=Release
