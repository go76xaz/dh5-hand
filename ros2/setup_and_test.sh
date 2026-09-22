#!/usr/bin/env bash
#
# setup_and_test.sh - bring up the DH5 ROS 2 integration on a Linux PC and
# run a short smoke test: build the workspace, install the `dh5` Python
# library, launch dh5_controller_node, initialize the hand, and run through
# a couple of gestures.
#
# Usage:
#   ./setup_and_test.sh [options]
#
# Options (env vars or flags, flags win):
#   -p, --port PORT        Serial port the DH5 is on        (default: /dev/ttyUSB0)
#   -b, --baud RATE        Modbus baud rate                 (default: 115200)
#   -i, --modbus-id ID     Modbus slave id                   (default: 1)
#   -w, --workspace DIR    ROS 2 workspace root               (default: repo root's ros2_ws,
#                                                              created next to this script)
#   -m, --init-mode MODE   0b01 close / 0b10 open / 0b11 find-stroke (default: 3 = find stroke)
#       --skip-build       Reuse an existing colcon build, don't rebuild
#       --no-gestures      Only bring the node up + initialize, skip gesture tests
#   -h, --help             Show this help
#
# Requires: a sourced ROS 2 install (Humble/Iron/Jazzy...), colcon, and
# read/write access to the serial port (usually: `sudo usermod -aG dialout $USER`,
# then re-login).
#
# Run this script from anywhere; it locates the repo from its own path.

set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults / argument parsing
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PORT="${DH5_PORT:-/dev/ttyUSB0}"
BAUD="${DH5_BAUD:-115200}"
MODBUS_ID="${DH5_MODBUS_ID:-1}"
WORKSPACE="${DH5_WS:-${REPO_ROOT}/ros2_ws}"
INIT_MODE="${DH5_INIT_MODE:-3}"   # 0b11 = find-stroke, the safe default on first bring-up
SKIP_BUILD=0
RUN_GESTURES=1

usage() { sed -n '2,26p' "${BASH_SOURCE[0]}"; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--port)       PORT="$2"; shift 2 ;;
        -b|--baud)       BAUD="$2"; shift 2 ;;
        -i|--modbus-id)  MODBUS_ID="$2"; shift 2 ;;
        -w|--workspace)  WORKSPACE="$2"; shift 2 ;;
        -m|--init-mode)  INIT_MODE="$2"; shift 2 ;;
        --skip-build)    SKIP_BUILD=1; shift ;;
        --no-gestures)   RUN_GESTURES=0; shift ;;
        -h|--help)       usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

log()  { printf '\n\033[1;34m[setup]\033[0m %s\n' "$*"; }
ok()   { printf '\033[1;32m[ ok ]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[fail]\033[0m %s\n' "$*" >&2; exit 1; }

NODE_PID=""
cleanup() {
    if [[ -n "${NODE_PID}" ]] && kill -0 "${NODE_PID}" 2>/dev/null; then
        log "Stopping dh5_controller_node (pid ${NODE_PID})..."
        kill "${NODE_PID}" 2>/dev/null || true
        wait "${NODE_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# 1. Sanity checks
# ---------------------------------------------------------------------------
log "Checking prerequisites"

if [[ -z "${ROS_DISTRO:-}" ]]; then
    # Try the common install locations before giving up.
    for candidate in /opt/ros/*/setup.bash; do
        [[ -f "${candidate}" ]] || continue
        log "Sourcing ${candidate}"
        # shellcheck disable=SC1090
        source "${candidate}"
        break
    done
fi
[[ -n "${ROS_DISTRO:-}" ]] || die "No ROS 2 environment found. Install ROS 2 and/or source its setup.bash first."
ok "ROS 2 distro: ${ROS_DISTRO}"

command -v colcon >/dev/null 2>&1 || die "colcon not found (apt install python3-colcon-common-extensions)."
command -v python3 >/dev/null 2>&1 || die "python3 not found."

if [[ ! -e "${PORT}" ]]; then
    warn "${PORT} does not exist yet. Is the DH5 plugged in? Continuing anyway."
elif [[ ! -r "${PORT}" || ! -w "${PORT}" ]]; then
    warn "No read/write access to ${PORT}. If this fails, add yourself to the 'dialout' group: sudo usermod -aG dialout \$USER (then re-login)."
fi

# ---------------------------------------------------------------------------
# 2. Install the `dh5` Python library into the environment ROS will run in
# ---------------------------------------------------------------------------
log "Installing the dh5 Python library (editable)"
python3 -m pip show dh5 >/dev/null 2>&1 && CURRENT_DH5="already installed" || CURRENT_DH5="not installed"
log "dh5 package: ${CURRENT_DH5} - (re)installing from ${REPO_ROOT} to pick up local changes"
python3 -m pip install --user -e "${REPO_ROOT}" -q || die "pip install of the dh5 library failed."
ok "dh5 library installed."

# ---------------------------------------------------------------------------
# 3. Build the ROS 2 workspace
# ---------------------------------------------------------------------------
mkdir -p "${WORKSPACE}/src"
if [[ ! -e "${WORKSPACE}/src/dh5_controller" ]]; then
    ln -s "${REPO_ROOT}/ros2/dh5_controller" "${WORKSPACE}/src/dh5_controller"
fi
if [[ ! -e "${WORKSPACE}/src/dh5_interfaces" ]]; then
    ln -s "${REPO_ROOT}/ros2/dh5_interfaces" "${WORKSPACE}/src/dh5_interfaces"
fi

if [[ "${SKIP_BUILD}" -eq 1 && -f "${WORKSPACE}/install/setup.bash" ]]; then
    log "Skipping build, reusing existing install at ${WORKSPACE}/install"
else
    log "Building workspace at ${WORKSPACE} (colcon build)"
    ( cd "${WORKSPACE}" && colcon build --symlink-install --packages-select dh5_interfaces dh5_controller ) \
        || die "colcon build failed."
    ok "Build complete."
fi

# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"

# ---------------------------------------------------------------------------
# 4. Launch the controller node
# ---------------------------------------------------------------------------
log "Starting dh5_controller_node (port=${PORT} baud=${BAUD} modbus_id=${MODBUS_ID})"
LOG_FILE="$(mktemp -t dh5_controller_XXXX.log)"
ros2 launch dh5_controller controller.launch.py \
    port:="${PORT}" baud_rate:="${BAUD}" modbus_id:="${MODBUS_ID}" \
    > "${LOG_FILE}" 2>&1 &
NODE_PID=$!
log "Node PID ${NODE_PID}, logging to ${LOG_FILE}"

log "Waiting for dh5/initialize service to come up..."
for _ in $(seq 1 60); do
    if ! kill -0 "${NODE_PID}" 2>/dev/null; then
        cat "${LOG_FILE}" >&2
        die "dh5_controller_node exited early. See log above."
    fi
    if ros2 service list 2>/dev/null | grep -q '/dh5/initialize'; then
        ok "Node is up."
        break
    fi
    sleep 1
done
ros2 service list 2>/dev/null | grep -q '/dh5/initialize' || die "Timed out waiting for the node's services."

# ---------------------------------------------------------------------------
# 5. Initialize the hand
# ---------------------------------------------------------------------------
log "Initializing the hand (mode=${INIT_MODE}) - this homes every axis and can take a while"
INIT_OUT="$(ros2 service call /dh5/initialize dh5_interfaces/srv/Initialize "{mode: ${INIT_MODE}}" --timeout 60 2>&1)" \
    || die "initialize call failed:\n${INIT_OUT}"
echo "${INIT_OUT}"
echo "${INIT_OUT}" | grep -q "success=True" || die "Hand failed to initialize (see response above)."
ok "Hand initialized."

log "Faults after init:"
ros2 service call /dh5/get_faults std_srvs/srv/Trigger "{}" || true

if [[ "${RUN_GESTURES}" -eq 0 ]]; then
    log "Skipping gesture tests (--no-gestures)."
    log "Node is running; press Ctrl+C to stop."
    wait "${NODE_PID}"
    exit 0
fi

# ---------------------------------------------------------------------------
# 6. Gesture smoke tests
# ---------------------------------------------------------------------------
call_trigger() {
    local service="$1"
    log "Calling dh5/${service} ..."
    local out
    out="$(ros2 service call "/dh5/${service}" std_srvs/srv/Trigger "{}" --timeout 30 2>&1)" || {
        warn "${service} call errored:\n${out}"
        return 1
    }
    echo "${out}"
    if echo "${out}" | grep -q "success=True"; then
        ok "${service} OK"
    else
        warn "${service} reported failure (see above)."
    fi
}

call_pinch() {
    local variant="$1" width="$2"
    log "Calling dh5/two_finger_pinch (axis_mode=${variant}, width=${width}) ..."
    local out
    out="$(ros2 service call /dh5/two_finger_pinch dh5_interfaces/srv/TwoFingerPinch \
        "{width: ${width}, axis_mode: '${variant}'}" --timeout 30 2>&1)" || {
        warn "two_finger_pinch call errored:\n${out}"
        return 1
    }
    echo "${out}"
    if echo "${out}" | grep -q "success=True"; then
        ok "two_finger_pinch(${variant}, width=${width}) OK"
    else
        warn "two_finger_pinch(${variant}, width=${width}) reported failure (see above)."
    fi
}

log "=== Gesture test sequence ==="

call_trigger open_hand
sleep 1

call_trigger point
sleep 1

call_trigger open_hand
sleep 1

call_trigger round_grip
sleep 1

call_trigger open_hand
sleep 1

call_pinch axis2 25    # thumb-to-index, fully open pinch
sleep 1
call_pinch axis2 0     # closed pinch
sleep 1

call_trigger open_hand
sleep 1

call_trigger wink
sleep 1

log "One last state snapshot (dh5/AxisInfos, one message):"
timeout 3 ros2 topic echo /dh5/AxisInfos --once || warn "No AxisInfos message received."

ok "Gesture test sequence complete."
log "Node is still running (PID ${NODE_PID}), log at ${LOG_FILE}. Press Ctrl+C to stop it."
wait "${NODE_PID}"
