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
FRAME_MODEL="Tools/autotest/models/quad_2kg.json"
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
NO_OBSTACLES=""
GOAL_MIN=""
GOAL_MAX=""
GOAL_RADIUS=""
ENT_COEF=""
MAX_STEPS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --mission)       MISSION="$2";              shift 2 ;;
        --timesteps)     TIMESTEPS="$2";            shift 2 ;;
        --lr)            LR="$2";                   shift 2 ;;
        --resume)        RESUME="$2";               shift 2 ;;
        --sitl-only)     SITL_ONLY=true;            shift ;;
        --no-rebuild)    NO_REBUILD="--no-rebuild"; shift ;;
        --no-obstacles)  NO_OBSTACLES="--no-obstacles"; shift ;;
        --goal-min)      GOAL_MIN="--goal-min $2";  shift 2 ;;
        --goal-max)      GOAL_MAX="--goal-max $2";  shift 2 ;;
        --goal-radius)   GOAL_RADIUS="--goal-radius $2"; shift 2 ;;
        --ent-coef)      ENT_COEF="--ent-coef $2";  shift 2 ;;
        --max-steps)     MAX_STEPS="--max-steps $2"; shift 2 ;;
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

# Generate sysid_params.txt from quad_2kg.json (single source of truth for physical params)
FRAME_MODEL_ABS="$ARDUPILOT_DIR/$FRAME_MODEL"
mkdir -p /tmp/sitl_cowork
python3 - "$FRAME_MODEL_ABS" /tmp/sitl_cowork/sysid_params.txt <<'PYEOF'
import json, sys
src, dst = sys.argv[1], sys.argv[2]
with open(src) as f:
    d = json.load(f)
lines = [
    f"MASS={d['mass']}",
    f"IXX={d['moment_inertia'][0]}",
    f"IYY={d['moment_inertia'][1]}",
    f"IZZ={d['moment_inertia'][2]}",
    f"THROTTLE_HOVER={d['hoverThrOut']}",
]
with open(dst, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"✅ sysid_params.txt generated from {src}")
PYEOF

# Copy lqr_gains.txt to SITL working directory so Mode 99 loads DARE gains instead of heuristic
LQR_GAINS_SRC="$ARDUPILOT_DIR/ArduCopter/lqr_gains.txt"
if [ -f "$LQR_GAINS_SRC" ]; then
    cp "$LQR_GAINS_SRC" "$WORKSPACE_DIR/lqr_gains.txt"
    cp "$LQR_GAINS_SRC" /tmp/sitl_cowork/lqr_gains.txt
    echo "✅ lqr_gains.txt copied to SITL working directory"
else
    echo "⚠️  lqr_gains.txt not found at $LQR_GAINS_SRC — Mode 99 will use heuristic gains"
fi

echo ""
echo "Starting SITL (arducopter binary directly, port $SITL_PORT)..."

# Save SITL command for drift-triggered restart by gym_env
# Must cd to ARDUPILOT_DIR so that relative model path (Tools/autotest/models/...) resolves correctly
cat > /tmp/sitl_mode99_cmd.sh << SITLEOF
cd "$ARDUPILOT_DIR" && "$ARDUPILOT_DIR/build/sitl/bin/arducopter" \
    --model "+:$FRAME_MODEL" \
    --speedup "$SITL_SPEEDUP" \
    --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$PARAM_FILE" \
    --sim-address=127.0.0.1 \
    -I0
SITLEOF

cd "$ARDUPILOT_DIR" && "$ARDUPILOT_DIR/build/sitl/bin/arducopter" \
    --model "+:$FRAME_MODEL" \
    --speedup "$SITL_SPEEDUP" \
    --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$PARAM_FILE" \
    --sim-address=127.0.0.1 \
    -I0 \
    >/tmp/sitl_mode99.log 2>&1 \
    &
SITL_PID=$!
echo "  SITL PID: $SITL_PID"
echo "$SITL_PID" > /tmp/sitl_mode99.pid

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
    # Convert to absolute path (handles both relative-to-workspace and relative-to-rl_training)
    RESUME_ABS="$(realpath "$WORKSPACE_DIR/$RESUME" 2>/dev/null)"
    if [ ! -f "$RESUME_ABS" ] && [ ! -f "${RESUME_ABS}.zip" ]; then
        RESUME_ABS="$(realpath "$RESUME" 2>/dev/null || echo "$RESUME")"
    fi

    # ── Validate action space before launching SITL ─────────────────────────────
    echo "  Checking action space of resume model..."
    RESUME_CHECK="$RESUME_ABS"
    [[ "$RESUME_CHECK" != *.zip ]] && RESUME_CHECK="${RESUME_CHECK}.zip"
    ACTION_DIM=$(python3 -c "
import zipfile, json, sys
try:
    with zipfile.ZipFile('$RESUME_CHECK') as z:
        data = json.loads(z.read('data'))
    asp = data['action_space']
    # SB3 stores action_space as a dict with _shape or directly
    shape = asp.get('_shape') or asp.get('shape') or []
    print(shape[0] if shape else '?')
except Exception as e:
    print('?')
" 2>/dev/null)
    ENV_DIM=$(python3 -c "
import sys; sys.path.insert(0, '.')
from ardupilot_gym_env import ArduPilotMode99Env
import inspect, re
src = inspect.getsource(ArduPilotMode99Env.__init__)
# action_space の定義以降で最初に出る shape=(N,) を取得
m = re.search(r'action_space.*?shape=\((\d+),\)', src, re.DOTALL)
print(m.group(1) if m else '?')
" 2>/dev/null)
    echo "  Model action_dim=$ACTION_DIM  Env action_dim=$ENV_DIM"
    if [ "$ACTION_DIM" != "?" ] && [ "$ENV_DIM" != "?" ] && [ "$ACTION_DIM" != "$ENV_DIM" ]; then
        echo "❌ Action space mismatch! Model=$ACTION_DIM, Env=$ENV_DIM"
        echo "   Use a checkpoint that matches the current environment (action_dim=$ENV_DIM)."
        echo "   Latest checkpoints:"
        ls -lt "$TRAINING_DIR/models/"*_steps.zip 2>/dev/null | head -5 | awk '{print "     " $NF}'
        exit 1
    fi
    echo "  ✅ Action space OK (dim=$ENV_DIM)"
    # ────────────────────────────────────────────────────────────────────────────

    RESUME_ARG="--resume $RESUME_ABS"
    echo "  Resuming from: $RESUME_ABS"
fi

python3 -u train_mode99_rl.py \
    --mode train \
    --mission "$MISSION" \
    --timesteps "$TIMESTEPS" \
    --lr "$LR" \
    $RESUME_ARG \
    $NO_OBSTACLES \
    $GOAL_MIN \
    $GOAL_MAX \
    $GOAL_RADIUS \
    $ENT_COEF \
    $MAX_STEPS
