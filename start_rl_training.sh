#!/bin/bash
# Quick Start: RL Training with PyBullet in Docker
# Usage: bash start_rl_training.sh

set -e

echo "======================================================================"
echo "RL TRAINING QUICK START"
echo "======================================================================"
echo ""

# Check if Docker is running
if ! docker ps > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker first."
    exit 1
fi
echo "✅ Docker is running"

# Check if image exists
if ! docker images | grep -q "autonomous_drone_sim"; then
    echo "⚠️  Docker image not found. Building..."
    docker compose build drone_sim
fi
echo "✅ Docker image ready"

# Start containers
echo ""
echo "Starting Docker containers..."
docker compose up -d drone_sim tensorboard jupyter

# Wait for containers to be ready
sleep 5

# Check container status
if docker ps | grep -q "drone_sim"; then
    echo "✅ drone_sim container running"
else
    echo "❌ drone_sim container failed to start"
    docker logs drone_sim --tail 20
    exit 1
fi

echo ""
echo "======================================================================"
echo "ENVIRONMENT READY!"
echo "======================================================================"
echo ""
echo "Access points:"
echo "  - TensorBoard: http://localhost:6006"
echo "  - Jupyter:     http://localhost:8888"
echo "  - Grafana:     http://localhost:3000"
echo ""
echo "======================================================================"
echo "STEP 1: Verify Installation"
echo "======================================================================"
echo ""
echo "Checking Python packages..."

docker exec drone_sim python3 << 'EOF'
import sys

packages = {
    'pybullet': 'PyBullet',
    'stable_baselines3': 'Stable-Baselines3',
    'gymnasium': 'Gymnasium',
    'torch': 'PyTorch',
    'drone_gym': 'drone_gym'
}

missing = []
installed = []

for pkg, name in packages.items():
    try:
        __import__(pkg)
        module = sys.modules[pkg]
        version = getattr(module, '__version__', 'unknown')
        installed.append(f"✅ {name}: {version}")
    except ImportError:
        missing.append(f"❌ {name}")

for pkg in installed:
    print(pkg)

if missing:
    print("\nMissing packages:")
    for pkg in missing:
        print(pkg)
    print("\nInstall with:")
    print("  docker exec -it drone_sim bash")
    print("  pip3 install pybullet stable-baselines3[extra] gymnasium[all]")
    sys.exit(1)
else:
    print("\n✅ All packages installed!")
    sys.exit(0)
EOF

VERIFY_STATUS=$?

echo ""
if [ $VERIFY_STATUS -eq 0 ]; then
    echo "======================================================================"
    echo "STEP 2: Start Training"
    echo "======================================================================"
    echo ""
    echo "Option A: Quick test (5 minutes)"
    echo "  docker exec -it drone_sim python3 /workspace/scripts/training/train_ppo.py --timesteps 10000"
    echo ""
    echo "Option B: Full training (20-24 hours)"
    echo "  docker exec -it drone_sim python3 /workspace/scripts/training/train_ppo.py --timesteps 1000000 --n-envs 8"
    echo ""
    echo "Option C: Interactive mode"
    echo "  docker exec -it drone_sim bash"
    echo "  cd /workspace"
    echo "  python3 scripts/training/train_ppo.py --timesteps 1000000"
    echo ""
    echo "======================================================================"
    echo "✅ READY TO TRAIN!"
    echo "======================================================================"
else
    echo "⚠️  Some packages missing. Install them first (see above)."
fi

echo ""
echo "For detailed guide, see: RL_TRAINING_GUIDE.md"
echo ""
