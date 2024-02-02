#!/bin/bash
set -e

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
        --merge-install \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic \
        --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release \
        # --packages-select piezobot_hardware_interface
                # --parallel-workers 1 \
