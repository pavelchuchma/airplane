#!/bin/zsh
# Launches Cockpit View with the project venv and default parameters
# (OTG camera "AV TO USB2.0", MAVLink on UDP :14445).
# Extra arguments are passed through, e.g.: ./run.sh --fullscreen --fov 45
cd "$(dirname "$0")"
exec ./.venv/bin/python cockpit_view.py "$@"
