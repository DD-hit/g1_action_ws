#!/usr/bin/env bash
# Source before the first ROS participant is created. Keep explicit overrides.
# Host setup: net.core.rmem_max and net.core.wmem_max >= 16777216.
# Keep RGB-D traffic off the robot's SDK network. This affects ROS only;
# the SDK participant still uses its configured domain 0 / eth0 transport.
# ROS setup files can already export ROS_LOCALHOST_ONLY=0. Use a project
# override so that the system default cannot silently disable isolation.
# Set G1_ROS_LOCALHOST_ONLY=0 on all action-library peers for remote access.
export ROS_LOCALHOST_ONLY=${G1_ROS_LOCALHOST_ONLY:-1}
if [[ -z "${CYCLONEDDS_URI:-}" ]]; then
  G1_TRANSPORT_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
  export CYCLONEDDS_URI="file://${G1_TRANSPORT_ROOT}/src/g1_actions/config/cyclonedds.xml"
  unset G1_TRANSPORT_ROOT
fi
