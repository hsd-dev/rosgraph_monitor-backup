#!/usr/bin/env bash
set -e

# setup ros environment
source "graph_ws/devel/setup.bash"
exec "$@"