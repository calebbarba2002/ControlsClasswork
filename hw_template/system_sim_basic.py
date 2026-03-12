"""
═══════════════════════════════════════════════════════════════════════════════
HOMEWORK TEMPLATE - BASIC SIMULATION (Chapters 8,11)
═══════════════════════════════════════════════════════════════════════════════

WHAT THIS SCRIPT DOES:
This is a complete control system simulation with 5 steps:
1. Create the physical system (e.g., robot arm, mass, pendulum)
2. Create a controller (the "brain" that computes control inputs)
3. Define a reference signal (what you WANT the system to do)
4. Run the simulation (see what ACTUALLY happens)
5. Visualize results (plots and/or animation)

EXPECTED OUTPUT:
- Plots showing state (position, velocity) vs time
- Plots showing control input vs time
- Animation of the physical system (optional)

TIME TO COMPLETE: ~30 minutes
DIFFICULTY: ⭐⭐ (Moderate - mainly filling in parameters)

═══════════════════════════════════════════════════════════════════════════════
"""

# ═══════════════════════════════════════════════════════════════════════════════
# IMPORTS
# ═══════════════════════════════════════════════════════════════════════════════
# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common, A_arm

# ↑ ALWAYS USE THIS IMPORT PATTERN!
# - "common" gives you: SignalGenerator, run_simulation, etc.
# - "A_arm" gives you: Dynamics, ControllerPD, Visualizer, params, etc.
#
# For your homework, replace A_arm with:
#   D_mass, E_blockbeam, F_vtol, etc.


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 1: Create the Physical System
# ═══════════════════════════════════════════════════════════════════════════════
# This creates a "virtual" version of your system that follows the physics
# equations you derived in Chapter 2-3.

arm = A_arm.Dynamics(alpha=0.0)
# ↑ Creates the arm dynamics model
#
# PARAMETER: alpha (parameter uncertainty)
#   - alpha = 0.0:  Perfect model (mass, length, etc. are exactly right)
#   - alpha = 0.2:  Model parameters could be off by ±20%
#
# WHY USE UNCERTAINTY?
#   Real systems don't match models perfectly! Testing with alpha > 0 checks
#   if your controller is "robust" (still works even with model errors).
#
# TIP: Start with alpha=0.0, get it working, then test with alpha=0.2

# FOR YOUR HOMEWORK: Replace with your system
# mass = D_mass.Dynamics(alpha=0.0)
# blockbeam = E_blockbeam.Dynamics(alpha=0.0)


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 2: Create the Controller
# ═══════════════════════════════════════════════════════════════════════════════
# The controller is the "brain" - it looks at where you are vs. where you
# want to be, and decides what control input to apply.

controller = A_arm.ControllerPD(use_feedback_linearization=True)
# ↑ Creates a PD (Proportional-Derivative) controller
#
# WHAT IS PD CONTROL?
#   u = kp * error - kd * velocity
#   - Pushes harder when you're FAR from target (proportional)
#   - Pushes less when you're moving FAST (derivative - acts like damping)
#
# PARAMETER: use_feedback_linearization
#   - True:  Cancels nonlinear terms (gravity, etc.) - works for large angles
#   - False: Linearizes around equilibrium - only works for small angles
#
# The gains (kp, kd) are computed automatically in __init__() based on
# desired pole locations or rise time. See the controller file for details.

# FOR YOUR HOMEWORK: Use the appropriate controller for the chapter
# Chapter 8:    controller = D_mass.ControllerPD()   # Assumes access to full state
# Chapters 9-10: controller = D_mass.ControllerPID() # Adds integral + dirty derivative
# Chapter 11:    controller = D_mass.ControllerSS()  # State-space control


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 3: Create Reference Signal (What You Want)
# ═══════════════════════════════════════════════════════════════════════════════
# The reference signal tells the controller where you WANT the system to go.
# Think of it like a "command", "desired", or "setpoint" signal.

theta_ref = common.SignalGenerator(amplitude=np.radians(50), frequency=0.05)
# ↑ Creates a signal that oscillates between +50° and -50°
#
# PARAMETERS:
#   - amplitude: How far to move (here: 50° converted to radians)
#   - frequency: How fast to change (here: 0.05 Hz = one cycle every 20 seconds)
#
# SIGNAL TYPES (call with time t):
#   - theta_ref.square(t):   Square wave (jump between +/- amplitude)
#   - theta_ref.step(t):     Step input (jump from 0 to amplitude at t=0)
#   - theta_ref.sawtooth(t): Ramp up and down
#   - theta_ref.sin(t):      Sinusoidal (smooth oscillation)
#
# WHY USE SQUARE WAVES?
#   They're challenging! The controller has to respond to sudden changes.
#   If it works for square waves, it'll work for smoother signals.
#
# UNITS WARNING! ⚠️
#   ALWAYS use radians for angles: np.radians(degrees)
#   Numpy trig functions use radians, not degrees!

# FOR YOUR HOMEWORK: Adjust amplitude/frequency as needed
# z_ref = common.SignalGenerator(amplitude=1.0, frequency=0.05)  # mass: 1 meter


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 4: Run the Simulation
# ═══════════════════════════════════════════════════════════════════════════════
# This is where the magic happens! The simulation loop repeatedly:
#   1. Gets desired position from reference signal
#   2. Controller computes control input based on current state
#   3. Physics updates the system state
#   4. Repeat for t_final seconds

time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
    arm,  # The physical system to simulate
    [theta_ref],  # List of reference signals (one per output)
    controller,  # The controller to use
    controller_input="state",  # Controller has access to full state
    t_final=20,  # Simulate for 20 seconds
    dt=A_arm.params.ts,  # Time step (usually 0.01s)
)
# ↑ Runs the simulation and returns time histories
#
# RETURNS (7 things total, we only need first 4 here):
#   time:    Array of time values [0.00, 0.01, 0.02, ..., 20.0]
#   x_hist:  State history [position, velocity] at each time
#   u_hist:  Control input history [force/torque] at each time
#   r_hist:  Reference history [desired position] at each time
#   *_:      Other stuff we don't need yet (observers, disturbances)
#
# The `*_` syntax means "there are more return values but ignore them"
# (Later homework will use all 7 return values)
#
# KEY PARAMETERS:
#   Three approaches across the chapters:
#
#   - "state" (Chapter 8):
#       Controller sees FULL state vector x (all positions & velocities)
#       Assumption: Perfect sensors measure everything
#       Controller method: update_with_state(r, x)
#
#   - "measurement" (Chapters 9-10):
#       Controller only sees position (y), estimates velocity numerically
#       Uses "dirty derivative" (numerical differentiation)
#       Controller methods: update_with_measurement(r, y) → computes ẏ internally
#
#   - "measurement" (Chapters 12-14):
#       Controller only sees measurements (y), uses observer for state estimation
#       Proper observer design with Luenberger observer
#       Controller method: update_with_measurement(r, y) with self.observer
#
# OPTIONAL PARAMETERS (not used here):
#   input_disturbance:  Add external forces (wind, friction)
#   output_noise:       Add sensor noise (imperfect measurements)

# FOR YOUR HOMEWORK: Same structure, just change system name
# time, x_hist, u_hist, r_hist, *_ = common.run_simulation(
#     mass, [z_ref], controller, controller_input="state",
#     t_final=20, dt=D_mass.params.ts
# )


# ═══════════════════════════════════════════════════════════════════════════════
# STEP 5: Visualize Results
# ═══════════════════════════════════════════════════════════════════════════════
# Now let's see how well the controller worked!

viz = A_arm.Visualizer(time, x_hist, u_hist, r_hist)
# ↑ Creates a visualization object with all the data

viz.plot()
# ↑ Shows static plots:
#   - Position vs time (does it follow the reference?)
#   - Velocity vs time (is it smooth?)
#   - Control input vs time (does it saturate? is it reasonable?)

# viz.animate()
# ↑ Shows animated visualization of the physical system moving
#   UNCOMMENT THIS to see the arm actually move!
#   (Comment out viz.plot() if you only want animation)

# FOR YOUR HOMEWORK: Same pattern
# viz = D_mass.Visualizer(time, x_hist, u_hist, r_hist)
# viz.plot()


# ═══════════════════════════════════════════════════════════════════════════════
# WHAT TO LOOK FOR IN THE PLOTS
# ═══════════════════════════════════════════════════════════════════════════════
"""
GOOD CONTROL SYSTEM:
✓ Position closely follows reference (small error)
✓ Smooth response, no crazy oscillations
✓ Control input is reasonable (doesn't hit limits constantly)
✓ Settles quickly after reference changes (fast rise time)
✓ Little to no overshoot

BAD CONTROL SYSTEM:
❌ Large error (doesn't reach target)
❌ Oscillates wildly (too much gain)
❌ Sluggish response (too little gain)
❌ Overshoot > 10% (underdamped)
❌ Control input constantly saturated (unrealistic)

DEBUGGING TIPS:
1. Controller does nothing?
   → Check that gains (kp, kd) are not zero
   → Print gains in controller __init__()
   
2. System unstable (blows up)?
   → Check pole locations (must have negative real parts!)
   → Try smaller gains (more conservative poles)
   → Check sign of error (should be: ref - actual)
   
3. Wrong direction?
   → Check sign of control law
   → Check sign of dynamics equations
   
4. Slow response?
   → Increase natural frequency wn (decrease rise time tr)
   → Move poles further left in complex plane
   
5. Oscillation/overshoot?
   → Increase damping ratio (zeta closer to 1.0)
   → Move poles closer to real axis

6. Not reaching target?
   → Need integral control (PID, not PD)
   → Or check equilibrium compensation (u_eq)
"""


# ═══════════════════════════════════════════════════════════════════════════════
# PYTHON NOTES FOR NON-EXPERTS
# ═══════════════════════════════════════════════════════════════════════════════
"""
NUMPY ARRAYS:
- Use np.array([1, 2, 3]) not regular lists [1, 2, 3]
- Access elements: x[0], x[1], not x.first, x.second
- Slice arrays: x[0:2] gets first two elements
- Math works element-wise: x * 2 doubles all elements

RADIANS vs DEGREES:
- ALWAYS use radians: theta = np.radians(45)  # 45° → radians
- Convert back: degrees = np.degrees(theta)
- Common mistake: Using 45 directly instead of np.radians(45)

IMPORTS:
- "from X import Y" means "get Y from package X"
- Use: A_arm.Dynamics(), not just Dynamics()
- This avoids confusion when multiple systems exist

RETURN VALUES:
- Multiple returns: a, b, c = function()  # gets all 3
- Ignore extras: a, b, *_ = function()    # only get first 2

COMMENTS:
- # single line comment
- Triple quotes for multi-line docstrings (like this!)
"""
