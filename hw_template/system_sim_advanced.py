"""
═══════════════════════════════════════════════════════════════════════════════
HOMEWORK TEMPLATE - ADVANCED SIMULATION WITH OBSERVERS (Chapters 12-14)
═══════════════════════════════════════════════════════════════════════════════

WHAT'S DIFFERENT FROM BASIC HOMEWORK?
This simulation is more realistic:
- You can only MEASURE some states (e.g., position), not all (e.g., no velocity)
- Controller uses an OBSERVER to estimate unmeasured states
- External DISTURBANCES affect the system (wind, friction, unknown forces)
- Sensors have NOISE (measurements aren't perfect)

This is closer to real-world control systems!

EXPECTED OUTPUT:
- Plots comparing actual states vs. estimated states (xhat)
- Plots showing disturbance and estimated disturbance (dhat)
- Control performance despite uncertainty and noise

TIME TO COMPLETE: ~45 minutes
DIFFICULTY: ⭐⭐⭐ (Moderate-Hard - requires observer design)

═══════════════════════════════════════════════════════════════════════════════
"""

# ═══════════════════════════════════════════════════════════════════════════════
# IMPORTS
# ═══════════════════════════════════════════════════════════════════════════════
import numpy as np
from case_studies import common, A_arm


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 1: Create the Physical System (Same as Basic)
# ═══════════════════════════════════════════════════════════════════════════════
arm = A_arm.Dynamics(alpha=0.2)
# ↑ Note: alpha=0.2 means ±20% parameter uncertainty
#   This tests ROBUSTNESS - controller must work even with model mismatch!


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 2: Create Advanced Controller (With Observer!)
# ═══════════════════════════════════════════════════════════════════════════════
controller = A_arm.ControllerSSIDO(separate_integrator=False)
# ↑ This is a state-space controller with:
#   - Integral action (I): Eliminates steady-state error
#   - Observer (O): Estimates unmeasured states
#   - Disturbance observer (DO): Estimates unknown external forces
#
# CONTROLLER TYPES (by chapter):
#   Ch 12: ControllerSSIO  - State-space + Integral + Observer
#   Ch 13: ControllerSSIDO - Add disturbance observer
#   Ch 14: ControllerLQRIDO - Optimal (LQR) version
#
# WHAT'S AN OBSERVER?
#   A "software sensor" - uses the model and measurements to estimate
#   states you can't measure directly (like velocity from position).
#
# WHY SEPARATE INTEGRATOR?
#   separate_integrator=False: Augment state with integrator (simpler)
#   separate_integrator=True:  Keep integrator separate (textbook approach)
#   Both work; False is more common in practice.


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 3: Create Reference Signal (Same as Basic)
# ═══════════════════════════════════════════════════════════════════════════════
theta_ref = common.SignalGenerator(amplitude=np.radians(50), frequency=0.05)


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 4: Create Disturbances and Noise (NEW!)
# ═══════════════════════════════════════════════════════════════════════════════

# External disturbance (constant force pushing on system)
d_force = np.array([0.5])
# ↑ This is an unknown external torque (0.5 N⋅m) acting on the arm
#   Like wind pushing on your robot, or friction you didn't model
#   The controller doesn't know about this - must handle it anyway!
#
# WHY TEST WITH DISTURBANCES?
#   Real systems have unknown forces. Good controllers reject disturbances!

# Sensor noise
theta_noise = common.SignalGenerator(amplitude=0.001)
# ↑ Adds random noise to measurements (±0.001 radians ≈ ±0.06°)
#   Real sensors aren't perfect - they have noise!
#
# WHY TEST WITH NOISE?
#   Ensures controller doesn't overreact to measurement errors


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 5: Run Advanced Simulation
# ═══════════════════════════════════════════════════════════════════════════════
time, x_hist, u_hist, r_hist, xhat_hist, d_hist, dhat_hist = common.run_simulation(
    arm,
    [theta_ref],
    controller,
    controller_input="measurement",  # ← KEY DIFFERENCE! Only see measurements
    input_disturbance=d_force,  # ← External force
    output_noise=[theta_noise],  # ← Sensor noise
    t_final=20,
    dt=A_arm.params.ts,
)
# ↑ Now we capture ALL 7 return values (not just first 4)
#
# KEY DIFFERENCES FROM BASIC SIMULATION:
#
# 1. controller_input="measurement" (not "state")
#    - Controller only sees y (measurements), not x (full state)
#    - Must use observer to estimate missing states
#    - More realistic!
#
# 2. input_disturbance=d_force
#    - Adds constant external force to control input
#    - Controller must compensate for this unknown force
#    - Tests "disturbance rejection"
#
# 3. output_noise=[theta_noise]
#    - Adds random noise to each measurement
#    - List with one generator per output (here: just theta)
#    - For multi-output: [noise1, noise2, ...]
#
# RETURN VALUES (now using all 7):
#   time:      Time array
#   x_hist:    TRUE state (what's actually happening)
#   u_hist:    Control input history
#   r_hist:    Reference history
#   xhat_hist: ESTIMATED state (what observer thinks is happening)  ← NEW!
#   d_hist:    Disturbance history (what's pushing on system)       ← NEW!
#   dhat_hist: ESTIMATED disturbance (what observer thinks)         ← NEW!
#
# COMPARISON:
#   Plot x_hist vs xhat_hist to see how good your observer is!
#   Plot d_hist vs dhat_hist to see if you're estimating disturbance correctly


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 6: Visualize Advanced Results
# ═══════════════════════════════════════════════════════════════════════════════
viz = A_arm.Visualizer(time, x_hist, u_hist, r_hist, xhat_hist, d_hist, dhat_hist)
# ↑ Now passing ALL the data including observer estimates

viz.plot()
# ↑ Plots will now show:
#   - Actual state (solid line)
#   - Estimated state (dashed line) ← Watch how close they are!
#   - Reference (what you want)
#   - Control input
#   - Actual disturbance vs estimated disturbance

# viz.animate()


# ═══════════════════════════════════════════════════════════════════════════════
# WHAT TO LOOK FOR IN ADVANCED PLOTS
# ═══════════════════════════════════════════════════════════════════════════════
"""
GOOD OBSERVER DESIGN:
✓ Estimated state (xhat) quickly converges to actual state (x)
✓ Small estimation error even with noise
✓ Estimated disturbance (dhat) close to actual (d)
✓ Position still tracks reference despite disturbances
✓ Transient error at beginning, then good tracking

BAD OBSERVER DESIGN:
❌ Estimated state doesn't match actual state
❌ Oscillations in xhat (observer poles too fast)
❌ Sluggish observer (poles too slow)
❌ Noise amplification (derivative gain too high)
❌ Control performance degrades vs. basic simulation

OBSERVER DESIGN TIPS:
1. Time-scale separation principle:
   - Make observer poles 5-10x faster than controller poles
   - If controller tr = 0.5s, then observer tr = 0.05s
   - Observer should track faster than controller needs
   
2. Too fast observer:
   - Amplifies noise (chattery control)
   - Might oscillate
   - Solution: Make observer poles slower
   
3. Too slow observer:
   - Lags behind actual state
   - Poor control performance
   - Solution: Make observer poles faster
   
4. Disturbance observer:
   - Disturbance pole even slower than state observer
   - Disturbances change slowly (low frequency)
   - Typical: around -5 or slower

DEBUGGING:
1. Observer not converging?
   → Check observability (use cnt.ctrb(A.T, C.T))
   → Check observer gain matrix L
   → Print observer poles (should be negative and larger in magnitude than controller poles!)
   
2. Control worse than basic simulation?
   → Observer too slow (increase observer bandwidth)
   → Noise too high (decrease noise or filter it with low-pass filter)
   
3. Oscillations?
   → Observer poles too fast (back off)
   → Reduce derivative gain in observer
   
4. Not rejecting disturbance?
   → Need disturbance observer (DO)
   → Check disturbance observer pole location
"""


# ═══════════════════════════════════════════════════════════════════════════════
# UNDERSTANDING OBSERVERS (SIMPLE EXPLANATION)
# ═══════════════════════════════════════════════════════════════════════════════
"""
PROBLEM:
You can measure position (theta) but not velocity (thetadot).
But your controller NEEDS access to velocity (or all states) to work well.
What do you do?

SOLUTION: OBSERVER (State Estimator)
An observer is like a "parallel simulation" that runs alongside your real system:

1. Start with an initial guess: xhat = [0, 0]
2. Apply same control input to simulated model: xhat_dot = A*xhat + B*u
3. Compare simulated output to real measurement: error = y - C*xhat
4. Correct the estimate: xhat_new = xhat + L * error
   (L is the "observer gain" - how much to trust the error)

The observer "learns" the true state by comparing its prediction with reality!

WHY IT WORKS:
- Model is good → prediction close to reality → small corrections
- Model is off → large error → big corrections to compensate
- Over time, xhat converges to x (actual state)

DISTURBANCE OBSERVER:
Same idea, but estimating unknown forces instead of states:
- Treat disturbance as an extra state (slowly varying)
- Estimate it like any other state
- Use estimate to cancel disturbance in control law

"""


# ═══════════════════════════════════════════════════════════════════════════════
# TYPICAL DEVELOPMENT WORKFLOW
# ═══════════════════════════════════════════════════════════════════════════════
"""
STEP-BY-STEP PROCESS:

1. Start with working basic controller (Ch 8-11)
   - Make sure it works without observer
   - Get good pole locations / rise time
   
2. Design observer (Ch 12)
   - Choose observer poles (5-10x faster than controller)
   - Implement update_with_measurement()
   - Test with measurement input (no disturbance yet)
   
3. Add integral action (if not already there)
   - Augment state with integrator state
   - Eliminates steady-state error
   
4. Add disturbance (Ch 13)
   - Test with constant disturbance
   - Should still track (thanks to integrator)
   
5. Add disturbance observer (Ch 13-14)
   - Augment state with disturbance state
   - Choose disturbance pole (slower than states)
   - Feed dhat into control law
   
6. Add noise and uncertainty (Ch 14)
   - Test with alpha > 0 (parameter uncertainty)
   - Add sensor noise
   - Should still work!
   
7. Tune and iterate
   - Adjust observer poles if needed
   - Balance performance vs. noise sensitivity
   - Compare to basic simulation

EVALUATION CRITERIA:
✓ Tracks reference despite disturbances
✓ Observer estimates converge to true states
✓ Smooth control (not chattery from noise)
✓ Robust to parameter uncertainty (alpha=0.2)
"""
