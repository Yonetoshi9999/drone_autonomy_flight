# Quick Reference Guide - RL Training

**Last Updated:** 2026-02-15 03:00 JST
**Training Status:** ✅ ACTIVE
**Started:** 2026-02-14 17:49 JST
**Expected Completion:** ~2026-02-15 18:00 JST

---

## 🚀 Training Status

### Check if Running
```bash
docker ps | grep drone_sim
```

### Watch Progress (Live)
```bash
docker exec -it drone_sim tail -f /workspace/data/training_output.log
```

---

## 📊 Monitoring

### TensorBoard (Best Option)
```
http://localhost:6006
```

### Progress CSV
```bash
docker exec drone_sim cat /workspace/data/logs/20260214_174923/progress.csv
```

---

## 💾 Checkpoints Location
```
/workspace/data/checkpoints/ppo_PyBulletDrone-v0_20260214_174923/
- best_model/best_model.zip  ← Use this one!
```

---

## 📈 Expected Performance

| Timesteps | Time | Success Rate |
|-----------|------|--------------|
| 50k | 3h | ~20% |
| 500k | 12h | ~70% |
| 1M | 24h | ~90% |

---

## 🔗 Quick Links

- TensorBoard: http://localhost:6006
- Jupyter: http://localhost:8888
- Grafana: http://localhost:3000

---

**Time remaining:** ~15 hours
