# Template Files for Student Implementations

This directory contains heavily-annotated template files to help you implement your own case studies (D_mass, E_blockbeam, F_vtol, etc.).

## 🎯 How to Use These Templates

### Step 1: Copy the Template
```bash
# Example: Creating D_mass dynamics
cp _TEMPLATE/dynamics_template.py D_mass/dynamics.py
```

### Step 2: Rename the Class
```python
# In your new file, change:
class YourSystemDynamics(DynamicsBase):
# To:
class MassDynamics(DynamicsBase):
```

### Step 3: Fill in the TODOs
Each template has clear TODO sections marked with:
- `# TODO: ...` comments
- `None` placeholders to replace
- Examples of what to put there

### Step 4: Test and Iterate
Run your homework file and check the output!

## 📄 What's in Each Template

### `dynamics_template.py` - The Physics

Implements how your system behaves over time.

**Two methods to fill in:**
1. `f(x, u)` - The differential equations (from textbook Ch 2-3)
2. `h()` - What you can measure (usually just position)

**Key sections:**
- Parameter loading with randomization
- Step-by-step f() implementation guide
- Common mistakes highlighted
- Examples for different system types

**Use this when**: Starting any new case study

---

### `controller_template.py` - The Controller or Feedback Law

Implements an example PD controller to calculate control input.

**Key sections:**
- Two design approaches (pole placement vs. rise time)
- Gain computation
- Control law implementation
- Debugging tips

**Use this when**: Implementing controllers for chapters 7-8

**Note**: For later chapters (PID, state-space, observers), look at reference implementations and adapt this template.

---

### `params_template.py` - The Model and System Parameters

All the constants and parameters for your system.

**Organized by chapter:**
- Ch 2: Physical parameters (mass, spring constant, etc.)
- Ch 3: Initial conditions (starting position, velocity)
- Ch 4: Equilibrium points
- Ch 5: Transfer function coefficients
- Ch 6: State-space matrices (A, B, C, D)
- Ch 8: Control limits (max force/torque)

**Use this when**: Starting any case study (fill in progressively as you advance through chapters)

---

## 🎓 Learning Progression

### Chapter 7-8: PD Control
Use: `params_template.py`, `dynamics_template.py`, `controller_template.py` and `update_with_state()` function in your controller.

### Chapter 9-10: PID Control
Reuse dynamics, create new PID controller that uses `update_with_measurement()` which requires us to take numerical derivatives of some states. 

### Chapters 11-12: State-Space
Reuse dynamics, create new state-space controller that uses `update_with_state()`.

### Chapter 13-14: Observers
Extend controllers to implement `update_with_measurement()` again, estimating 
state using observers. 

## ⚠️ Important Reminders

### DO:
- ✓ Use the templates as starting points
- ✓ Read ALL the comments (they contain important info!)
- ✓ Compare with reference implementations (A_arm, B_pendulum, C_satellite)
- ✓ Test frequently as you fill in sections
- ✓ If `update_with_measurement` isn't working, revert back to using full state to test your controller implementation first. 
- ✓ Use radians for angles (`np.radians(deg)`)
- ✓ Use `x` in your dynamics function `f(x,u)` (not `self.state`)
- ✓ Return only numpy arrays (not floats or lists)

### DON'T:
- ✗ Skip reading the docstrings
- ✗ Copy-paste without understanding
- ✗ Overuse auto-completion of code 


## 🔗 Related Resources

- **Getting Started**: See [STUDENT_QUICKSTART.md](../../STUDENT_QUICKSTART.md) in repository root
- **Homework Examples**: See `hw_template/` directory
- **Reference Implementations**: See `A_arm/`, `B_pendulum/`, `C_satellite/`

## 🐛 Troubleshooting

**"I copied the template but it doesn't work"**
→ Did you fill in ALL the TODOs? Use the debugger to help you figure out where it is failing!

**"What's the difference between template files and reference case studies (i.e. A, B, and C)?"**
→ Templates are teaching tools (heavy comments). References are working code (lighter comments). Start with templates, compare to reference case studies.

**"Should I use/reuse helper classes that I put in the `control` folder? or write the math directly in each controller?"**
→ Either works! For learning, writing directly might help understanding. For cleaner code, use helpers.

**"My f(x,u) isn't working"**
→ Check: (1) Using `x` not `self.state`? (2) Returning numpy array? (3) Correct order in xdot?

**"System is unstable!"**
→ Poles should be NEGATIVE! `p1 = -2.0`, not `p1 = 2.0`, double check your closed-loop poles. 

## 🎉 You've Got This!

These templates are here to help you focus on **control theory** rather than fighting with Python syntax. When you get stuck:

1. Read the template comments carefully
2. Compare to a reference implementation
3. Check the Quick Start guide
4. Print values to debug
5. Ask for help if still stuck!

Good luck! 🚀
