# 5M Timestep Training - Live Status

**Status:** ✅ TRAINING IN PROGRESS
**Started:** 2026-02-14 19:18:30 JST
**Expected Completion:** ~2026-02-15 00:30 JST (approximately 5 hours)
**Current Time:** 2026-02-15 03:18 JST

---

## Training Configuration

| Parameter | Value |
|-----------|-------|
| **Total Timesteps** | 5,000,000 (5x previous training) |
| **Parallel Envs** | 4 |
| **Learning Rate** | 0.0003 |
| **Batch Size** | 64 |
| **Seed** | 42 (for reproducibility) |
| **Expected Duration** | ~5 hours |

---

## Why 5M Timesteps?

### Previous Training (1M steps):
- ❌ Reward: 39.4 (too low)
- ❌ Success rate: ~20-30%
- ❌ High variance: +/- 171
- ⚠️ Inconsistent behavior

### Expected with 5M steps:
- ✅ Reward: 70-80+
- ✅ Success rate: 60-80%
- ✅ Lower variance (more consistent)
- ✅ Reliable navigation

**5x more training = Much better performance!**

---

## Progress Tracking

### Quick Status Check
```bash
# Check if running
docker ps | grep drone_sim

# Latest log lines
docker exec drone_sim tail -20 /workspace/data/training_5M_output.log
```

### Live Monitoring
```bash
# Watch training progress (live)
docker exec -it drone_sim tail -f /workspace/data/training_5M_output.log

# Press Ctrl+C to exit
```

### TensorBoard (Best for monitoring)
```
http://localhost:6006

Refresh the page to see new data from 5M training run
```

---

## Expected Progress Timeline

| Time from Start | Timesteps | Expected Reward | Status |
|-----------------|-----------|-----------------|--------|
| 0 min | 0 | Random | Started ✅ |
| 1 hour | 1M | 30-40 | Better than 1st run |
| 2 hours | 2M | 50-60 | Noticeable improvement |
| 3 hours | 3M | 60-70 | Good performance |
| 4 hours | 4M | 70-80 | Very good |
| **5 hours** | **5M** | **75-85** | **Target** ✅ |

---

## Checkpoints

### Location
```
/workspace/data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/
```

### Checkpoint Schedule
Saves every 200,000 steps:
- ppo_model_200000_steps.zip (every ~1 hour)
- ppo_model_400000_steps.zip
- ppo_model_600000_steps.zip
- ...
- ppo_model_5000000_steps.zip
- **best_model/best_model.zip** ⭐ (best evaluation)
- **final_model.zip** (at completion)

### Check Checkpoints
```bash
docker exec drone_sim ls -lh /workspace/data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/
```

---

## Monitoring Commands

### 1. Check Training Status
```bash
# Is it running?
docker exec drone_sim ps aux | grep train_ppo
```

### 2. View Latest Progress
```bash
# Last 50 lines of output
docker exec drone_sim tail -50 /workspace/data/training_5M_output.log
```

### 3. View Progress CSV
```bash
docker exec drone_sim cat /workspace/data/logs/20260214_191830/progress.csv | tail -5
```

### 4. TensorBoard Metrics
```
Open: http://localhost:6006

Key metrics to watch:
- rollout/ep_rew_mean → Should increase to 70-80
- rollout/ep_len_mean → Should increase
- train/loss → Should stabilize
```

---

## Performance Milestones

### ✅ Success Criteria (at 5M steps)

**Target Performance:**
- Episode Reward: > 70
- Success Rate: > 60%
- Episode Length: > 250 steps
- Reward Variance: < 100 (more consistent)

**If achieved:**
- Agent can navigate reliably
- Ready for deployment testing
- Can integrate with Mode 99

---

## What to Expect During Training

### Phase 1: Early Learning (0-1M steps, 0-1 hour)
- **Behavior:** Random exploration, frequent crashes
- **Reward:** -50 to 20
- **Episode Length:** 100-150 steps
- **Status:** Learning basic controls

### Phase 2: Skill Acquisition (1-3M steps, 1-3 hours)
- **Behavior:** Basic navigation, some goal reaching
- **Reward:** 20-60
- **Episode Length:** 150-200 steps
- **Status:** Learning to avoid obstacles

### Phase 3: Refinement (3-5M steps, 3-5 hours)
- **Behavior:** Consistent navigation, efficient paths
- **Reward:** 60-80
- **Episode Length:** 200-300 steps
- **Status:** Polishing policy, reducing failures

### Phase 4: Convergence (5M steps, ~5 hours)
- **Behavior:** Reliable goal reaching
- **Reward:** 70-85
- **Episode Length:** 250-350 steps
- **Status:** Near-optimal performance ✅

---

## Comparison with Previous Training

| Metric | 1M Training | 5M Training (Expected) |
|--------|-------------|------------------------|
| **Duration** | 59 min | ~5 hours |
| **Final Reward** | 39.4 | 70-80 |
| **Success Rate** | 20-30% | 60-80% |
| **Consistency** | High variance | Low variance |
| **Usability** | Proof of concept | Ready for testing |

---

## If Training Stops Early

### Check Status
```bash
# View last 100 lines
docker exec drone_sim tail -100 /workspace/data/training_5M_output.log

# Check for errors
docker exec drone_sim grep -i "error" /workspace/data/training_5M_output.log
```

### Resume Training
```bash
# Find latest checkpoint
docker exec drone_sim ls -lt /workspace/data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/

# Resume from checkpoint
docker exec -it drone_sim python3 << 'EOF'
from stable_baselines3 import PPO

# Load latest checkpoint (e.g., 2M steps)
model = PPO.load("data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/ppo_model_2000000_steps.zip")

# Continue training for remaining steps
model.learn(total_timesteps=3000000)  # 3M more to reach 5M total

# Save
model.save("data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/ppo_model_5000000_steps")
EOF
```

---

## After Training Completes

### 1. Evaluate Performance
```bash
docker exec -it drone_sim python3 << 'EOF'
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
import gymnasium as gym
import drone_gym

model = PPO.load("data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/best_model/best_model")
env = gym.make("PyBulletDrone-v0")

mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=50)
print(f"Mean Reward: {mean_reward:.2f} +/- {std_reward:.2f}")
env.close()
EOF
```

### 2. Visual Test
```bash
docker exec -it drone_sim python3 << 'EOF'
from stable_baselines3 import PPO
import gymnasium as gym
import drone_gym

model = PPO.load("data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/best_model/best_model")
env = gym.make("PyBulletDrone-v0", render_mode="human", gui=True)

obs, _ = env.reset()
for _ in range(3000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, _ = env.step(action)
    if done or truncated:
        print(f"Episode reward: {reward}")
        obs, _ = env.reset()
env.close()
EOF
```

### 3. Compare with 1M Training
```bash
# Download both models
docker cp drone_sim:/workspace/data/checkpoints/ppo_PyBulletDrone-v0_20260214_174923/best_model/best_model.zip ~/model_1M.zip
docker cp drone_sim:/workspace/data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/best_model/best_model.zip ~/model_5M.zip
```

---

## Troubleshooting

### Out of Memory
```bash
# Check container memory
docker stats drone_sim

# If needed, reduce parallel envs
# Edit and restart with: --n-envs 2
```

### Training Slow
```bash
# Check CPU usage
docker exec drone_sim top -bn1 | grep python

# Expected: ~400% CPU (4 cores)
```

### Checkpoints Not Saving
```bash
# Check disk space
docker exec drone_sim df -h /workspace/data/

# Each checkpoint: ~1.5 MB (very small)
```

---

## Quick Reference

```bash
# Status check
docker exec drone_sim tail -20 /workspace/data/training_5M_output.log

# Live monitoring
docker exec -it drone_sim tail -f /workspace/data/training_5M_output.log

# TensorBoard
http://localhost:6006

# Checkpoints
docker exec drone_sim ls -lh /workspace/data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/
```

---

## Files

### Training Data
- **Log:** `/workspace/data/training_5M_output.log`
- **Checkpoints:** `/workspace/data/checkpoints/ppo_PyBulletDrone-v0_20260214_191830/`
- **TensorBoard:** `/workspace/data/logs/20260214_191830/`

### Documentation
- **This file:** `TRAINING_5M_STATUS.md`
- **Previous 1M results:** `RL_TRAINING_STATUS_FINAL.md`
- **Session summary:** `SESSION_SUMMARY_FEB15.md`

---

**Training started:** 2026-02-14 19:18:30 JST
**Expected completion:** 2026-02-15 ~00:30 JST
**Check back in 5 hours for results!**

**Next milestone:** 1M checkpoint in ~1 hour (around 20:20 JST)
