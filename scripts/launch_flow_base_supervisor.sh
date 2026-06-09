#!/bin/bash
# Launch the FlowBase controller supervisor daemon on the base's Raspberry Pi.
#
# One-time install (run on the Pi):
#   scp thirdparty/i2rt/scripts/flow-base-supervisor.service \
#       i2rt@172.6.2.20:/etc/systemd/system/flow-base-supervisor.service
#   ssh i2rt@172.6.2.20 'sudo systemctl daemon-reload && \
#       sudo systemctl enable --now flow-base-supervisor'
#
# The supervisor itself never edits flow_base_controller.py; it only spawns/kills
# it over a portal RPC (see i2rt/flow_base/flow_base_supervisor.py).

# Repo root and venv (override via env if your layout differs).
FLOW_BASE_ROOT="${FLOW_BASE_ROOT:-/home/i2rt/JH_i2rt}"
FLOW_BASE_VENV="${FLOW_BASE_VENV:-/home/i2rt/JH_i2rt/JH_i2rt}"
FLOW_BASE_CHANNEL="${FLOW_BASE_CHANNEL:-can0}"

source "${FLOW_BASE_VENV}/bin/activate"
PYTHONPATH="${FLOW_BASE_ROOT}" python -m i2rt.flow_base.flow_base_supervisor \
    --venv "${FLOW_BASE_VENV}" \
    --root "${FLOW_BASE_ROOT}" \
    --channel "${FLOW_BASE_CHANNEL}"
