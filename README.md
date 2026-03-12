# Control Systems Case Studies - Course Repository

Educational Python codebase for a feedback control systems textbook course.

## 📚 Overview

This repository contains complete implementations of 7 physical systems (arm, pendulum, satellite, mass, block-and-beam, VTOL, hummingbird) with progressively sophisticated controllers (PD → PID → state-space → observer-based → LQR). We also include frequency-based controllers as a separate topic. 

## 🎓 Students start here:

[STUDENT_QUICKSTART.md](STUDENT_QUICKSTART.md)

This comprehensive guide covers:
- Getting started (installation, first simulation)
- Repository structure and navigation
- Homework workflow (step-by-step)
- Critical conventions and common errors
- Debugging tips and success criteria

### Quick Links
- **Templates**: [src/case_studies/_TEMPLATE/](src/case_studies/_TEMPLATE/) - Annotated skeletons for your implementations
- **Homework Examples**: [hw_template/](hw_template/) - Fully-commented example files to perform simulation
- **Reference Systems or Case Studies**: [A_arm](src/case_studies/A_arm/), [B_pendulum](src/case_studies/B_pendulum/), [C_satellite](src/case_studies/C_satellite/) - Complete working examples

### Your Assignments and Labs
You will implement dynamics, measurements, and controllers for these systems:
- **D_mass**: Mass-spring-damper system
- **E_blockbeam**: Ball-and-beam balancing
- **F_vtol**: Vertical takeoff aircraft
- **H_hummingbird**: Half-quadcopter with three degrees of freedom

## 🛠️ Installation

```bash
# Clone repository
git clone https://github.com/byu-controlbook/byu_controlbook_public.git
cd byu_controlbook_public

# Create virtual environment (recommended)
python -m venv .venv
.venv\Scripts\activate  # Windows
# source .venv/bin/activate  # Mac/Linux

# Install package
pip install -e .
```

## 🚀 Quick Example to Test Installation

```python
import numpy as np
from case_studies import common, A_arm

# Create system and controller
arm = A_arm.Dynamics()
controller = A_arm.ControllerPD()
ref = common.SignalGenerator(amplitude=np.radians(30), frequency=0.05)

# Run simulation
time, x, u, r, *_ = common.run_simulation(
    arm, [ref], controller, 
    controller_input="state", 
    t_final=10
)

# Visualize
A_arm.Visualizer(time, x, u, r).plot()
```

## 📂 Repository Structure

```
controls-course/
├── src/case_studies/           # Main package
│   ├── common/                 # Base classes and utilities
│   ├── _TEMPLATE/              # Templates for student implementations
│   ├── control/                # Helper classes (PD, PID, observers, state-space controllers)
│   ├── A_arm/                  # Reference: Robot arm
│   ├── B_pendulum/             # Reference: Inverted pendulum on cart
│   ├── C_satellite/            # Reference: Satellite
│   ├── D_mass/                 # Homework: Mass-spring-damper
│   ├── E_blockbeam/            # Homework: Block-and-beam
│   ├── F_vtol/                 # Homework: VTOL aircraft
│   └── H_hummingbird/          # Lab: Hummingbird
│
├── hw08/, hw09/, .../hw14/     # Homework simulation scripts by chapter
├── hw_template/                # Annotated homework examples
├── labs/                       # Lab (specific simulations or derivations)
│
└── Documentation files (this level)
```

## 📖 Learning Progression

### Chapters 2-6: Modeling
- Derive equations of motion
- Transfer functions
- State-variable and State-space representation
- Linearize dynamics


### Chapters 7-8: PD Control
- Pole placement
- Rise time and damping
- Saturation limits

### Chapters 9-10: PID Control
- Integral action
- Anti-windup
- Steady-state error elimination
- Digital implementation

### Chapter 11: State-Space Control
- Full state feedback

### Chapter 12: State-Space Control with Integrator
- Integral augmentation

### Chapters 13-14: Observers
- State estimation
- Disturbance observers
- Measurement feedback
- Robustness to noise and uncertainty

### Supplementary Material: Optimal Control
- LQR (optimal control)

## 🔑 Key Features

### For Learning
- **Progressive complexity**: From simple PD to advanced LQR with observers
- **Consistent structure**: Every case study follows the same pattern
- **Extensive documentation**: Templates, guides, and inline comments
- **Visual feedback**: Plots and animations for every system

### For Research/Extension
- **Modular design**: Easy to add new systems or controllers
- **Clean architecture**: Dynamics, controllers, and visualization separated
- **Parameter randomization**: Test robustness to model uncertainty
- **Observer framework**: Complete infrastructure for estimation

## 🤝 Contributing

If you find errors or have suggestions for improving the documentation or templates, please open an issue or email the instructor!

## 📄 License
MIT license

## 🙏 Acknowledgments

Based on textbook: https://drive.google.com/file/d/1OH6oSsbbdsxkY2CTMMxnchkWnNy_16zy/view?usp=sharing 

## 📞 Support

- **Students**: Start with [STUDENT_QUICKSTART.md](STUDENT_QUICKSTART.md)
- **Issues**: Open an issue on GitHub
- **Questions**: marc_killpack@byu.edu

---

**Happy controlling! 🎮🤖**
