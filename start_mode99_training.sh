#!/bin/bash
# Start Mode 99 RL Training
#
# Launches one ArduPilot SITL instance (speedup=5) then starts PPO training.
# SITL speedup=5 matches time_scale=5.0 in ardupilot_gym_env.py.
#
# Usage:
#   bash start_mode99_training.sh [--mission obstacle_avoidance|waypoint_navigation]
#                                 [--timesteps 1000000] [--lr 3e-4]
#                                 [--resume models/ppo_obstacle_avoidance_30000_steps.zip]
#                                 [--sitl-only] [--no-rebuild]

set -e

# ── Configurable paths ─────────────────────────────────────────────────────────
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
PARAM_FILE="$WORKSPACE_DIR/configs/ardupilot/params.parm"
TRAINING_DIR="$WORKSPACE_DIR/rl_training"
SITL_PORT=5760   # arducopter binary listens on 5760 directly (no MAVProxy needed)
SITL_SPEEDUP=5

# ── Argument parsing ───────────────────────────────────────────────────────────
MISSION="obstacle_avoidance"
TIMESTEPS=1000000
LR="3e-4"
RESUME=""
SITL_ONLY=false
NO_REBUILD=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --mission)    MISSION="$2";     shift 2 ;;
        --timesteps)  TIMESTEPS="$2";   shift 2 ;;
        --lr)         LR="$2";          shift 2 ;;
        --resume)     RESUME="$2";      shift 2 ;;
        --sitl-only)  SITL_ONLY=true;   shift ;;
        --no-rebuild) NO_REBUILD="--no-rebuild"; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ── Pre-flight checks ──────────────────────────────────────────────────────────
if [ ! -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]; then
    echo "❌ ArduPilot not found at $ARDUPILOT_DIR"
    echo "   Set ARDUPILOT_DIR env var or install ArduPilot."
    exit 1
fi

if [ ! -f "$PARAM_FILE" ]; then
    echo "❌ Param file not found: $PARAM_FILE"
    exit 1
fi

# ── Start SITL ─────────────────────────────────────────────────────────────────
echo "======================================================================"
echo "Mode 99 RL Training"
echo "======================================================================"
echo "  SITL port:    $SITL_PORT"
echo "  SITL speedup: ${SITL_SPEEDUP}x  (matches time_scale=5.0 in gym env)"
echo "  Mission:      $MISSION"
echo "  Timesteps:    $TIMESTEPS"
echo "  ArduPilot:    $ARDUPILOT_DIR"
echo "======================================================================"

echo ""
echo "Starting SITL (arducopter binary directly, port $SITL_PORT)..."
"$ARDUPILOT_DIR/build/sitl/bin/arducopter" \
    --model + \
    --speedup "$SITL_SPEEDUP" \
    --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$PARAM_FILE" \
    --sim-address=127.0.0.1 \
    -I0 \
    >/tmp/sitl_mode99.log 2>&1 \
    &
SITL_PID=$!
echo "  SITL PID: $SITL_PID"

# Cleanup SITL on exit (Ctrl-C or normal exit)
cleanup() {
    echo ""
    echo "Stopping SITL (PID $SITL_PID)..."
    kill "$SITL_PID" 2>/dev/null || true
    wait "$SITL_PID" 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

if [ "$SITL_ONLY" = true ]; then
    echo ""
    echo "SITL started. Waiting... (Ctrl-C to stop)"
    wait "$SITL_PID"
    exit 0
fi

# ── Wait for SITL TCP port to open ─────────────────────────────────────────────
echo ""
echo "Waiting for SITL to accept connections on port $SITL_PORT..."
WAIT_MAX=120
WAITED=0
until nc -z 127.0.0.1 "$SITL_PORT" 2>/dev/null; do
    if [ $WAITED -ge $WAIT_MAX ]; then
        echo "❌ SITL did not open port $SITL_PORT within ${WAIT_MAX}s"
        echo "   Check SITL log: tail /tmp/sitl_mode99.log"
        exit 1
    fi
    sleep 1
    WAITED=$((WAITED + 1))
    echo -n "."
done
echo ""
echo "✅ SITL ready on port $SITL_PORT"

# ── Start RL training ──────────────────────────────────────────────────────────
echo ""
echo "Starting RL training..."
echo "  (Checkpoints saved to $TRAINING_DIR/models/)"
echo "  (Logs saved to       $TRAINING_DIR/logs/)"
echo ""

cd "$TRAINING_DIR"
RESUME_ARG=""
if [ -n "$RESUME" ]; then
    RESUME_ARG="--resume $RESUME"
    echo "  Resuming from: $RESUME"
fi

python3 train_mode99_rl.py \
    --mode train \
    --mission "$MISSION" \
    --timesteps "$TIMESTEPS" \
    --lr "$LR" \
    $RESUME_ARG
