# Drone Physics Calibration Guide

## Problem Statement

We've been **guessing** at drone physics parameters (mass, PID gains) without measuring them. This leads to issues like:
- Drone slowly sinking (altitude loss)
- Oscillation or instability
- Inconsistent behavior

**Solution:** Data-driven calibration that measures actual parameters from the simulation.

---

## Quick Start

### 1. Run Calibration Tests

```bash
./start_simulation.sh drone 1 flat office --calibrate
```

This will:
- Measure actual mass from USD properties
- Run 3 automated tests (~113 seconds total)
  - **Test 1:** Free-fall (3s) - Verifies gravity/mass
  - **Test 2:** Hover force sweep (50s) - Finds optimal hover force
  - **Test 3:** PID tuning (60s) - Auto-tunes control gains
- Save results to `calibration_data.json`

### 2. Analyze Results

```bash
python3 calibrate_drone_physics.py --analyze calibration_data.json
```

This will:
- Process test data
- Calculate optimal parameters
- Generate `CALIBRATION_RESULTS.md` with recommendations

### 3. Apply Recommended Parameters

Open `CALIBRATION_RESULTS.md` and follow the instructions to update:
- `omniverse_sim.py` - Mass and force gains
- `robots/quadcopter/config.py` - Damping coefficients (if needed)

### 4. Verify

```bash
python3 test_drone_diagnostics.py
```

All tests should now **PASS**, especially "Altitude Hold/Stability".

---

## What Gets Measured

### Mass Measurement
- **Source:** USD physics properties (`robot.get_masses()`)
- **Verification:** Free-fall acceleration test
- **Expected:** 0.5-3.5 kg (Crazyflie scaled 5x)

### Hover Force
- **Method:** Test forces from 10-20N, find minimum stable hover
- **Expected:** Should be close to `m × g` (mass × gravity)
- **Accounts for:** Air resistance / damping

### PID Gains
- **Method:** Ziegler-Nichols auto-tuning
- **Process:**
  1. Test 6 different Kp values (5, 10, 15, 20, 25, 30)
  2. Find critical gain where oscillation starts
  3. Calculate optimal Kp, Ki, Kd from formula
- **Expected:** Kp ≈ 10-20, Ki ≈ 5-10, Kd ≈ 1-3

---

## Calibration Test Details

### Test 1: Free-Fall (3 seconds)
- **Purpose:** Verify mass by measuring acceleration
- **Method:** Disable control, let drone fall
- **Records:** Position, velocity at 60 Hz
- **Analysis:** Calculate acceleration from position changes
- **Expected:** a ≈ -9.81 m/s² (gravity)

### Test 2: Hover Force Sweep (50 seconds)
- **Purpose:** Find minimum force needed to hover
- **Method:** Test 10 force levels (10-20N), 5 seconds each
- **Records:** Applied force, position, velocity
- **Analysis:** Find force with minimum vertical drift
- **Expected:** Hover force ≈ measured mass × 9.81

### Test 3: PID Tuning (60 seconds)
- **Purpose:** Auto-tune PID gains for stable control
- **Method:** Test 6 Kp values (5-30), 10 seconds each
- **Target:** Maintain 2.0m altitude
- **Records:** Position, velocity, command
- **Analysis:** 
  - Count oscillations (zero-crossings)
  - Calculate standard deviation (stability)
  - Apply Ziegler-Nichols formula

---

## Understanding the Results

### Good Results
- **Mass:** Consistent with USD model
- **Hover Force:** Close to `m × g`
- **PID:** Shows progression from stable → oscillating → unstable
- **Tests Pass:** All diagnostic tests pass after applying parameters

### Bad Results (Troubleshooting)

**Problem:** Mass seems way off (e.g., 10kg or 0.01kg)
- Check USD file properties
- Verify scaling factor (currently 5x)
- Ensure `robot.get_masses()` works

**Problem:** Hover force >> `m × g`
- Check damping settings (should be ~0.01)
- Verify gravity constant (9.81 m/s²)
- Look for excessive air resistance

**Problem:** No oscillation in PID test
- Increase Kp range (test higher values)
- Check that target altitude != starting altitude
- Verify PID controller is actually running

---

## Parameters to Update

### 1. Mass (`omniverse_sim.py` line ~987)
```python
custom_rl_env.env_drone_mass = X.XXXX  # kg (from calibration)
```

### 2. Force Gain (`omniverse_sim.py` line ~116)
```python
kp_accel = XX.X  # From Ziegler-Nichols tuning
```

### 3. Damping (if needed, `robots/quadcopter/config.py` line ~17)
```python
linear_damping=0.XX,  # Adjust if hover force significantly > m×g
```

---

## Validation

After applying parameters, run:

```bash
python3 test_drone_diagnostics.py
```

**Expected Results:**
- ✅ Test 1: Odometry - PASS
- ✅ Test 2: Arming - PASS
- ✅ Test 3: **Altitude Hold/Stability (10 sec) - PASS** ← This was failing before!
- ✅ Test 4: Landing from Hover - PASS
- ✅ Test 5: Takeoff & Position Control - PASS
- ✅ Test 6: Land Service - PASS

The key test is **Test 3 (Altitude Hold)** - it should now show minimal altitude change (<5cm).

---

## Theory: Why This Works

### The Problem
We were using:
- **Guessed mass:** 1.5 kg (no measurement)
- **Guessed gain:** 15.0 (arbitrary, kept increasing)
- **Result:** Drone sinks because forces aren't calibrated

### The Solution
1. **Measure actual mass** from USD → Use correct `m` in `F = ma`
2. **Calculate hover force** → Know exactly how much force needed to counteract gravity
3. **Auto-tune PID** → Find gains that provide stable, responsive control

### Physics Equations
```
Hover: F_up = m × g + F_damping
Control: F = m × (Kp × v_error + g)  (simplified)
Velocity: v_new = v_current + (F/m) × dt
```

When we **measure** `m` instead of guessing, all these equations work correctly!

---

## Advanced: Re-Calibration

Re-run calibration if:
- USD model changes
- Scaling factor changes
- Damping/physics properties modified
- Tests start failing again

It only takes ~2 minutes and gives you accurate parameters every time.

---

## Files

- `calibrate_drone_physics.py` - Main calibration script
- `calibration_data.json` - Raw test data (generated during tests)
- `CALIBRATION_RESULTS.md` - Analysis and recommendations (generated after analysis)
- `calibration_test_plan.json` - Test configuration (generated for reference)

---

## Questions?

If calibration doesn't solve your issue:
1. Check logs during calibration - are tests actually running?
2. Verify `calibration_data.json` has data for all 3 tests
3. Check if analysis reports any warnings
4. Ensure parameters are actually applied to `omniverse_sim.py`

This is a **systematic, data-driven** approach - no more guessing!

