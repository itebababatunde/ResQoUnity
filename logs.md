
[TEST 1] Odometry Publishing
--------------------------------------------------------------------------------
✅ PASS: Odometry publishing
   Position: (-0.000, -0.000, 2.437)
ℹ️  INFO: Initial altitude: 2.437m

[TEST 2] Arm Service
--------------------------------------------------------------------------------
✅ PASS: Drone armed successfully
   Drone armed

[TEST 3] Altitude Hold After Arming
--------------------------------------------------------------------------------
ℹ️  INFO: Position before: Z=2.438m
ℹ️  INFO: Waiting 5 seconds to check altitude stability...
ℹ️  INFO: Position after:  Z=2.438m
ℹ️  INFO: Altitude change: 0.000m
✅ PASS: Altitude stable (drift: 0.000m)

[TEST 4] Takeoff Service
--------------------------------------------------------------------------------
✅ PASS: Takeoff command accepted
   Takeoff initiated to 1.5m

[TEST 5] Altitude Change After Takeoff
--------------------------------------------------------------------------------
ℹ️  INFO: Altitude before takeoff: 2.111m
ℹ️  INFO: Waiting 10 seconds for takeoff...
ℹ️  INFO: Altitude after takeoff: 2.111m
ℹ️  INFO: Altitude gain: +0.000m
❌ FAIL: No altitude change detected (0.000m)
ℹ️  INFO: Physics reports position change but actual position unchanged
ℹ️  INFO: Possible: Velocities not being applied to physics state

[TEST 6] Position Control (Horizontal Movement)
--------------------------------------------------------------------------------
ℹ️  INFO: Position before: (-0.000, -0.000, 2.111)
ℹ️  INFO: Commanding position: (1.0, 0.5, 2.6)
ℹ️  INFO: Waiting 15 seconds to reach target...
ℹ️  INFO: Position after: (-0.000, -0.000, 2.111)
ℹ️  INFO: Position error: (1.000, 0.500, 0.500) = 1.225m
❌ FAIL: Did not reach target (error: 1.225m > 0.5m threshold)
ℹ️  INFO: No movement detected - drone is stuck
ℹ️  INFO: Check: Are velocities being applied? Is physics running?

[TEST 7] Land Service
--------------------------------------------------------------------------------
✅ PASS: Land command accepted
   Landing sequence initiated

================================================================================
  TEST SUMMARY
================================================================================

Total Tests: 7
✅ Passed: 5
❌ Failed: 2

Detailed Results:
  ✅ Odometry: PASS
  ✅ Arm Service: PASS
  ✅ Altitude Hold: PASS
  ✅ Takeoff Service: PASS
  ❌ Takeoff Altitude Gain: FAIL
  ❌ Position Control: FAIL
  ✅ Land Service: PASS

⚠️  2 test(s) failed. Review output above for debugging info.
