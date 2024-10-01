#!/bin/bash

PARAMS=(
/ekf_localization/base_link_frame
/ekf_localization/debug
/ekf_localization/frequency
/ekf_localization/initial_estimate_covariance
/ekf_localization/initial_state
/ekf_localization/map_frame
/ekf_localization/odom0
/ekf_localization/odom0_config
/ekf_localization/odom_frame
/ekf_localization/print_diagnostics
/ekf_localization/sensor_timeout
/ekf_localization/transform_time_offset
/ekf_localization/transform_timeout
/ekf_localization/two_d_mode
/ekf_localization/world_frame
)

for param in "${PARAMS[@]}"
do
  rosparam delete $param
done
