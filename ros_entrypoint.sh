#!/bin/bash
set -e

if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

if [ -f /opt/conda/etc/profile.d/conda.sh ]; then
  source /opt/conda/etc/profile.d/conda.sh
fi

if [ "${1:-}" = "bash" ]; then
  exec bash -l
fi

exec "$@"
