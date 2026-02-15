#!/bin/bash
# Training Monitor Script
# Usage: ./monitor_training.sh

echo "============================================================"
echo "Training Monitor - Autonomous Drone RL"
echo "============================================================"
echo ""

# Find latest training directory
LATEST_DIR=$(docker exec drone_sim bash -c "ls -t /workspace/data/logs/ | grep PPO | head -1")

if [ -z "$LATEST_DIR" ]; then
    echo "❌ No training found"
    exit 1
fi

echo "📊 Monitoring: $LATEST_DIR"
echo ""

# Check if training process is running
PROCESS=$(docker exec drone_sim bash -c "ps aux | grep train_pybullet_rl.py | grep -v grep")

if [ -z "$PROCESS" ]; then
    echo "⚠️  Training process not running"
else
    echo "✓ Training process: RUNNING"

    # Get CPU and memory usage
    CPU=$(echo "$PROCESS" | awk '{print $3}')
    MEM=$(echo "$PROCESS" | awk '{print $4}')
    echo "  CPU: ${CPU}%"
    echo "  Memory: ${MEM}%"
fi

echo ""
echo "============================================================"
echo "Training Progress"
echo "============================================================"

# Try to read progress from TensorBoard logs or checkpoints
docker exec drone_sim bash -c "
    cd /workspace

    # Count checkpoints
    CHECKPOINT_DIR=data/checkpoints/$LATEST_DIR
    if [ -d \$CHECKPOINT_DIR ]; then
        NUM_CHECKPOINTS=\$(ls -1 \$CHECKPOINT_DIR/*.zip 2>/dev/null | wc -l)
        echo \"Checkpoints saved: \$NUM_CHECKPOINTS\"

        # Show latest checkpoint
        LATEST_CKPT=\$(ls -t \$CHECKPOINT_DIR/*.zip 2>/dev/null | head -1)
        if [ -n \"\$LATEST_CKPT\" ]; then
            echo \"Latest: \$(basename \$LATEST_CKPT)\"
        fi
    fi

    # Show log files
    LOG_DIR=data/logs/$LATEST_DIR
    if [ -d \$LOG_DIR ]; then
        LOG_SIZE=\$(du -sh \$LOG_DIR 2>/dev/null | cut -f1)
        echo \"Log size: \$LOG_SIZE\"
    fi
"

echo ""
echo "============================================================"
echo "How to View Detailed Progress"
echo "============================================================"
echo ""
echo "1. TensorBoard (Recommended):"
echo "   docker exec -it drone_sim tensorboard --logdir=/workspace/data/logs --host=0.0.0.0 --port=6006"
echo "   Then open: http://localhost:6006"
echo ""
echo "2. Check training logs:"
echo "   docker logs drone_sim | tail -50"
echo ""
echo "3. View checkpoints:"
echo "   docker exec drone_sim ls -lh /workspace/data/checkpoints/$LATEST_DIR/"
echo ""
echo "4. Check if still running:"
echo "   docker exec drone_sim ps aux | grep train_pybullet"
echo ""
echo "============================================================"
echo "Training will take approximately 1-2 hours"
echo "Current time: $(date)"
echo "============================================================"
