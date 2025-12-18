#!/bin/bash
# This script launches SteamVR in a clean, isolated environment
# to prevent conflicts with sourced environments like ROS.

cd /opt/steamvr

# Use 'env -i' to start with a completely empty (ignored) environment.
# We then add back a few essential variables (PATH, HOME, DISPLAY)
# that are required for most applications to run correctly.
env -i bash -c " \
    export PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'; \
    export HOME='$HOME'; \
    export DISPLAY=\"$DISPLAY\"; \
    ./start_steamvr_offline.sh \
"