#!/usr/bin/env bash
# Build the lean crazyflies-traj image (user = host uid/gid),
# then colcon-build ros2_ws in a throwaway container (ros2_ws/post_build.sh).

source setup.env

echo "Building image $RR_IMAGE_NAME:$RR_IMAGE_TAG (RR_ROS_DISTRO=$RR_ROS_DISTRO, uid $RR_USER_UID gid $RR_USER_GID)"

# --network=host: the build inherits the host DNS resolver (BuildKit has no --dns).
docker build --network=host \
    --build-arg="RR_USER_UID=$RR_USER_UID" \
    --build-arg="RR_USER_GID=$RR_USER_GID" \
    --build-arg="RR_USERNAME=$RR_USERNAME" \
    --build-arg="RR_ROS_DISTRO=$RR_ROS_DISTRO" \
    -t "$RR_IMAGE_NAME:$RR_IMAGE_TAG" \
    -t "$RR_IMAGE_NAME:latest" .
build_status=$?

if [ $build_status -eq 0 ] && [ -f "$(pwd)/ros2_ws/post_build.sh" ]; then
    echo "Build successful, running ros2_ws/post_build.sh..."
    docker run --rm --net=none \
        --user "$RR_USER_UID:$RR_USER_GID" \
        -v "$(pwd)/ros2_ws:/ros2_ws" \
        "$RR_IMAGE_NAME:$RR_IMAGE_TAG" \
        /bin/bash -c "source /opt/ros/${RR_ROS_DISTRO}/setup.bash && bash /ros2_ws/post_build.sh"
    build_status=$?
fi
exit $build_status
