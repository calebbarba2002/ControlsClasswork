# Student Quick Start Guide

Welcome to the Control Systems course code! This guide will help you navigate the codebase and complete your homework assignments.

## 📚 What You'll Find Here

This repository contains Python implementations of 7 physical systems with progressively sophisticated controllers:

### ✅ Reference Systems (Study These!)
- **A_arm**: Rotational arm (robot arm, single joint)
- **B_pendulum**: Inverted pendulum (classic control problem)
- **C_satellite**: Satellite attitude control

These are COMPLETE implementations - use them as references!

### 🛠️ Your Assignments
- **D_mass**: Mass-spring-damper system (YOU implement!)
- **E_blockbeam**: Ball-and-beam system (YOU implement!)
- **F_vtol**: Vertical takeoff and landing aircraft (YOU implement!)

### 📖 Controller Progression
As you progress through chapters, you'll implement increasingly sophisticated controllers:
1. **PD** (Proportional-Derivative) - Chapter 8: Full state feedback
2. **PID** (adds Integral action) - Chapters 9-10: Measurement + numerical derivative
3. **State-Space** (full state feedback) - Chapter 11: Matrix-based design
4. **Observer-based** (estimate unmeasured states) - Chapters 12-14: Proper state estimation
5. **LQR** (optimal control) - Advanced: Optimal gain selection

### 🎯 Three Feedback Approaches

**Approach 1 - Full State (Chapter 8):**
- Access to complete state: `x = [position, velocity]`
- Direct feedback: `u = kp*error - kd*velocity`
- Ideal but unrealistic (perfect sensors everywhere)

**Approach 2 - Numerical Derivative (Chapters 9-10):**
- Measure position only: `y = position`
- Estimate velocity: `velocity ≈ (position - position_prev) / dt`
- Called "dirty derivative" (amplifies noise!)
- Requires low-pass filtering

**Approach 3 - Observer-Based (Chapters 12-14):**
- Measure position only: `y = position`
- Observer estimates full state: `x̂ = [position_est, velocity_est]`
- Uses dynamics model + measurements
- Filters noise, can estimate disturbances

## 🚀 Getting Started (First 30 Minutes)

### 1. Install Dependencies

```bash
# Create a virtual environment (recommended)
python -m venv .venv
.venv\Scripts\activate  # Windows
# source .venv/bin/activate  # Mac/Linux

# Install the package
pip install -e .
```

### 2. Run Your First Simulation

```python
# Try this in Python or create a file:
from case_studies import common, A_arm
import numpy as np

# Create system and controller
arm = A_arm.Dynamics()
controller = A_arm.ControllerPD()
ref = common.SignalGenerator(amplitude=np.radians(30), frequency=0.05)

# Run simulation
time, x, u, r, *_ = common.run_simulation(
    arm, [ref], controller, controller_input="state", t_final=10
)

# Visualize
A_arm.Visualizer(time, x, u, r).plot()
```

If you see plots → Success! 🎉

### 3. Explore a Reference System

Look at [src/case_studies/A_arm/](src/case_studies/A_arm/):
- `params.py` - All the numbers (mass, length, etc.)
- `dynamics.py` - Physics equations
- `pd_controller.py` - Simple controller
- Run [hw08/arm_sim.py](hw08/arm_sim.py) to see it work!

## 📂 Understanding the Structure

```
src/case_studies/
├── common/                    ← Shared base classes (don't modify!)
│   ├── dynamics_base.py       → Template for all systems
│   ├── controller_base.py     → Template for all controllers
│   └── simulation.py          → Main simulation loop
│
├── _TEMPLATE/                 ← Copy these to get started!
│   ├── dynamics_template.py   → Skeleton for dynamics
│   ├── controller_template.py → Skeleton for controller
│   └── params_template.py     → Skeleton for parameters
│
├── A_arm/                     ← REFERENCE (complete example)
├── B_pendulum/                ← REFERENCE (complete example)
├── C_satellite/               ← REFERENCE (complete example)
│
├── D_mass/                    ← YOU COMPLETE THIS
├── E_blockbeam/               ← YOU COMPLETE THIS
└── F_vtol/                    ← YOU COMPLETE THIS

hw08/                          ← Homework simulations by chapter
hw09/
...
hw14/

hw_template/                   ← Annotated homework examples
├── system_sim_basic.py        → For chapters 8-11
└── system_sim_advanced.py     → For chapters 12-14
```

## 🎯 Typical Homework Workflow

### For Chapters 7-8 (PD Control)

**What you'll do**: Implement a PD controller for your assigned system

**Steps**:
1. **Fill in `params.py`** (Chapter 2-3 parameters)
   - Physical constants (mass, length, spring constant, etc.)
   - Initial conditions
   - Transfer function coefficients (from Chapter 5)

2. **Implement `dynamics.py`** (the physics)
   - Copy `_TEMPLATE/dynamics_template.py`
   - Fill in `f(x, u)` with differential equations
   - Fill in `h()` with measurement equation

3. **Implement `pd_controller.py`** (the brain)
   - Copy `_TEMPLATE/controller_template.py`
   - Compute PD gains from desired poles or rise time
   - Implement `update_with_state(r, x)`

4. **Test it!**
   - Copy `hw_template/system_sim_basic.py` to `hw08/`
   - Change `A_arm` to your system (e.g., `D_mass`)
   - Run and check plots

5. **Tune and iterate**
   - Adjust poles / rise time
   - Check tracking, overshoot, settling time

### For Chapters 12-14 (Observers)

**What's new**: Controller can only see measurements (not full state)

**Additional steps**:
4. **Design observer** (estimate unmeasured states)
   - Choose observer poles (5-10x faster than controller)
   - Implement `update_with_measurement(r, y)`
   - Return `(u, xhat)` or `(u, xhat, dhat)`

5. **Test with disturbances and noise**
   - Use `hw_template/system_sim_advanced.py`
   - Add `input_disturbance` and `output_noise`
   - Check that estimates converge

## 🔧 Essential Import Pattern

**ALWAYS use this pattern**:

```python
from case_studies import common, A_arm
# Then use: A_arm.Dynamics(), A_arm.ControllerPD(), etc.
```

**NOT this**:
```python
from case_studies.A_arm.dynamics import ArmDynamics  # ❌ Too specific
import case_studies  # ❌ Too vague
```

## 📐 Critical Conventions

### 1. State Vector Convention
State = [positions, velocities] (always in this order!)
```python
# For mass-spring:
x = [z, zdot]  # position, velocity

# For arm:
x = [theta, thetadot]  # angle, angular velocity
```

### 2. ALWAYS Use Radians (Not Degrees!)
```python
# ✓ Correct
theta = np.radians(45)  # Convert degrees to radians

# ✗ Wrong
theta = 45  # numpy functions expect radians!
```

### 3. Dynamics f(x,u) vs Output h()
```python
def f(self, x, u):
    """DON'T use self.state! Use the x parameter."""
    position = x[0]  # ✓ Use x
    velocity = x[1]
    # ... equations ...
    return xdot

def h(self):
    """DO use self.state here."""
    position = self.state[0]  # ✓ Use self.state
    return np.array([position])
```

**Why?** `f()` needs to work for ANY x values (RK4 integration uses intermediate values). `h()` measures the CURRENT state.

### 4. Return Arrays (Not Scalars)
```python
# ✓ Correct
return np.array([u])  # Return as array

# ✗ Wrong
return u  # Don't return scalar
```

## 🐛 Common Errors and Solutions

### "NotImplementedError: Must implement f(x,u)"
→ You haven't implemented the dynamics! See `_TEMPLATE/dynamics_template.py`

### System explodes (goes to infinity)
→ Check poles have **negative** real parts (stable!)
→ Print poles: `print(np.roots(desired_poles))`

### Wrong direction (moves away from target)
→ Check error sign: should be `error = ref - actual` (not reversed)
→ Check signs in dynamics equations

### Not reaching target (steady-state error)
→ Need integral action (use PID, not PD)
→ Or check equilibrium compensation (u_eq)

### Can't measure velocity but need it
→ Chapter 8: Assume you can measure it (idealized)
→ Chapters 9-10: Use numerical derivative (dirty derivative)
→ Chapters 12+: Use an observer to estimate it properly!

### Using degrees instead of radians
→ `theta = np.radians(45)` to convert
→ `np.sin()`, `np.cos()` expect radians!

## 📊 Understanding the Plots

After running `viz.plot()`, you'll see:

### State Plots
- **Solid line**: Actual state
- **Dashed line**: Reference (what you want)
- **Dotted line**: Estimated state (chapters 12+)

**Look for**:
- ✓ State follows reference closely
- ✓ Smooth response, no wild oscillations
- ✓ Fast rise time (reaches target quickly)
- ✓ Low overshoot (< 10%)

### Control Input Plot
**Look for**:
- ✓ Reasonable magnitude (not constantly saturated)
- ✓ Smooth (not chattery from noise)
- ✗ Hitting limits constantly = need to reduce gains or increase limits

### Observer Plots (Chapters 12+)
**Look for**:
- ✓ Estimated state (xhat) converges to actual state (x)
- ✓ Small estimation error even with noise
- ✗ Diverging estimates = observer not working

## 🎓 Learning Progression

### Chapter 8: PD Control Basics
**Goal**: Make system track a reference
**Concepts**: Poles, natural frequency, damping ratio
**Feedback**: Full state access (measure position AND velocity)
**Homework**: Implement PD controller with `update_with_state(r, x)`

### Chapters 9-10: Add Integral Action
**Goal**: Eliminate steady-state error
**New concept**: Integral gain, anti-windup, dirty derivative
**Feedback**: Measurement only (position) → estimate velocity numerically
**Homework**: Implement `update_with_measurement(r, y)` using numerical differentiation
**Key change**: Can only measure position, compute velocity via `(y - y_prev)/dt`

### Chapter 11: State-Space Control
**Goal**: More systematic design approach
**Concepts**: State-space representation (A, B, C, D matrices)
**Feedback**: Back to full state (for simplicity before observers)
**Homework**: Implement state-space controller

### Chapter 12: Observers
**Goal**: Estimate unmeasured states properly (no more dirty derivatives!)
**New concept**: Luenberger observer design, time-scale separation
**Feedback**: Measurement only → observer estimates full state from dynamics
**Homework**: Design observer that converges 5-10x faster than controller

### Chapter 13-14: Disturbance Rejection
**Goal**: Handle unknown external forces
**New concept**: Disturbance observer
**Homework**: Reject constant disturbances, handle sensor noise

## 🆘 Getting Help

### Check These First
1. **Reference implementations**: Look at A_arm, B_pendulum, C_satellite
2. **Templates**: `_TEMPLATE/` directory has detailed instructions
3. **Homework templates**: `hw_template/` has annotated examples
4. **Error messages**: Read the custom error messages - they tell you what's missing!

### Debug Checklist
- [ ] Printed parameters to verify they're loaded correctly?
- [ ] Checked that dynamics equations match textbook?
- [ ] Verified poles have negative real parts?
- [ ] Using radians (not degrees)?
- [ ] Returning numpy arrays (not scalars)?
- [ ] Checked matrix dimensions (A is n×n, B is n×m)?

### Still Stuck?
1. Run a reference system (A_arm) to verify your setup works
2. Print intermediate values (gains, poles, states) to debug
3. Start with small gains/slow poles, then increase gradually
4. Compare your code side-by-side with reference implementation

## 🎉 Success Criteria

Your implementation is working well when:

✅ System tracks reference with < 5% steady-state error  
✅ Rise time matches your design specs  
✅ Overshoot < 10%  
✅ Control input is reasonable (not constantly saturated)  
✅ Works with parameter uncertainty (alpha=0.2)  
✅ (Ch 12+) Observer estimates converge to true states  
✅ (Ch 13+) Rejects disturbances effectively  

## 💡 Pro Tips

1. **Start simple**: Get PD working before adding complexity
2. **Test incrementally**: Don't implement everything at once
3. **Use print statements**: Print gains, poles, intermediate values
4. **Compare to reference**: When stuck, compare to A_arm line-by-line
5. **Tune conservatively**: Start with slow poles, speed up gradually
6. **Visualize often**: Run simulations frequently to catch errors early
7. **Learn from plots**: They tell you what's wrong (oscillation, offset, etc.)

## 📚 Additional Resources

- **Copilot Instructions**: [.github/copilot-instructions.md](.github/copilot-instructions.md) - Architecture guide
- **Improvement Suggestions**: [STUDENT_CODING_IMPROVEMENTS.md](STUDENT_CODING_IMPROVEMENTS.md) - Detailed pedagogical notes
- **Control Library README**: [src/case_studies/control/README.md](src/case_studies/control/README.md) - Helper classes explained

---

**Good luck with your controls journey! 🚀**

*Remember: Control theory is about making systems do what you want. Start with the math from your textbook, implement it in code, simulate, and tune until it works. You've got this!*
