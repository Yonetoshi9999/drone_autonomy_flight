#!/bin/bash
# run_mode99.sh — Full Mode 99 LQI operation script
#
# Steps:
#   1. Compute LQR K gains from sysid_params.txt   (lqr_design.py)
#   2. Kill old SITL, clean eeprom.bin
#   3. Start ArduCopter SITL in background
#   4. Wait for SITL ready (port 5760 listening)
#   5. Run companion script (companion_mode99.py)
#
# Usage:
#   ./run_mode99.sh              # full run
#   ./run_mode99.sh --sitl-only  # start SITL only (no companion)
#   ./run_mode99.sh --gains-only # regenerate K gains only

set -e

# ── Paths ──────────────────────────────────────────────────────────────────
ARDUPILOT_DIR="$HOME/ardupilot"
SITL_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
PARAMS="$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm"
COWORK="/tmp/sitl_cowork"
SITL_LOG="$COWORK/sitl_run.txt"
COMPANION_DIR="$HOME/autonomous_drone_sim"
SYSID_FILE="$COWORK/sysid_params.txt"
LQR_GAINS_FILE="$COWORK/lqr_gains.txt"

# SITL home position (lat, lon, alt, heading)
HOME_POS="35.3629,138.7274,0,0"

MODE="full"
if [[ "$1" == "--sitl-only" ]]; then MODE="sitl"; fi
if [[ "$1" == "--gains-only" ]]; then MODE="gains"; fi

# ── Helpers ────────────────────────────────────────────────────────────────
ok()   { echo "[OK]  $*"; }
info() { echo "[--]  $*"; }
warn() { echo "[!!]  $*"; }
die()  { echo "[ERR] $*" >&2; exit 1; }

# ── Preflight checks ───────────────────────────────────────────────────────
echo "======================================================================"
echo " Mode 99 LQI Operation Script"
echo "======================================================================"

[ -f "$SITL_BIN" ]   || die "arducopter binary not found: $SITL_BIN"
[ -f "$PARAMS" ]     || die "default params not found: $PARAMS"
[ -d "$COWORK" ]     || { mkdir -p "$COWORK"; ok "Created $COWORK"; }
[ -f "$SYSID_FILE" ] || die "sysid_params.txt not found: $SYSID_FILE"

ok "Preflight checks passed"
info "SITL binary : $SITL_BIN"
info "Params file : $PARAMS"
info "Work dir    : $COWORK"
info "Sysid file  : $SYSID_FILE"
echo ""

# ── Step 1: Generate LQR gains ─────────────────────────────────────────────
echo "[1/4] Generating LQR K gains from sysid_params.txt ..."
python3 "$COMPANION_DIR/lqr_design.py" "$SYSID_FILE"
[ -f "$LQR_GAINS_FILE" ] || die "lqr_design.py did not produce $LQR_GAINS_FILE"
ok "K gains written to $LQR_GAINS_FILE"
echo ""

if [[ "$MODE" == "gains" ]]; then
    ok "Done (gains-only mode)"
    exit 0
fi

# ── Step 2: Kill old SITL, clean state ─────────────────────────────────────
echo "[2/4] Cleaning up old SITL processes and state ..."
pkill -9 -f 'arducopter.*model' 2>/dev/null && info "Killed old SITL" || true
sleep 1
rm -f "$COWORK/eeprom.bin"
ok "eeprom.bin removed (clean parameter state)"
echo ""

# ── Step 3: Start SITL ─────────────────────────────────────────────────────
echo "[3/4] Starting ArduCopter SITL ..."
info "Log: $SITL_LOG"
cd "$COWORK"
"$SITL_BIN" --model + --home "$HOME_POS" --defaults "$PARAMS" \
    >"$SITL_LOG" 2>&1 &
SITL_PID=$!
info "SITL PID: $SITL_PID"

# Wait for SITL to open port 5760 (up to 30s)
READY=0
for i in $(seq 1 30); do
    sleep 1
    if grep -q "Waiting for connection" "$SITL_LOG" 2>/dev/null; then
        READY=1
        break
    fi
    # Also accept if we can see the port is open
    if ss -tlnp 2>/dev/null | grep -q ':5760'; then
        READY=1
        break
    fi
done

if [ $READY -eq 0 ]; then
    warn "SITL did not reach 'Waiting for connection' within 30s"
    warn "Last lines of SITL log:"
    tail -10 "$SITL_LOG" | sed 's/^/    /'
    kill $SITL_PID 2>/dev/null || true
    die "SITL startup failed"
fi

# Confirm process is still alive
if ! kill -0 $SITL_PID 2>/dev/null; then
    warn "SITL process died! Last lines:"
    tail -10 "$SITL_LOG" | sed 's/^/    /'
    die "SITL crashed during startup"
fi

ok "SITL ready on port 5760 (PID $SITL_PID)"
echo ""

if [[ "$MODE" == "sitl" ]]; then
    ok "Done (sitl-only mode). Connect MAVProxy or companion manually."
    info "SITL log: $SITL_LOG"
    info "Stop with: kill $SITL_PID"
    exit 0
fi

# ── Step 4: Run companion ──────────────────────────────────────────────────
echo "[4/4] Running companion script ..."
info "Press Ctrl+C to abort"
echo ""
cd "$COMPANION_DIR"
python3 companion_mode99.py
COMPANION_EXIT=$?

echo ""
echo "======================================================================"
if [ $COMPANION_EXIT -eq 0 ]; then
    ok "Companion finished successfully"
else
    warn "Companion exited with code $COMPANION_EXIT"
fi

# Cleanup SITL
info "Stopping SITL (PID $SITL_PID) ..."
kill $SITL_PID 2>/dev/null || true
echo "======================================================================"
echo " SITL log: $SITL_LOG"
echo " LQR gains: $LQR_GAINS_FILE"
echo "======================================================================"
exit $COMPANION_EXIT
