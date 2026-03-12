"""
═══════════════════════════════════════════════════════════════════════════════
PARAMETERS FILE TEMPLATE FOR STUDENTS
═══════════════════════════════════════════════════════════════════════════════

INSTRUCTIONS:
1. Copy this file to your case study folder (e.g., D_mass/params.py)
2. Fill in the physical parameters from the problem statement
3. Complete each chapter's section as you progress through the course

ORGANIZATION:
This file is organized by TEXTBOOK CHAPTER - makes it easy to find what you
need for each homework assignment!

═══════════════════════════════════════════════════════════════════════════════
"""

import numpy as np


# ═══════════════════════════════════════════════════════════════════════════════
# Chapter 2: PHYSICAL PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════
# These are the "real" properties of your system from the problem statement.
# Get these from the textbook problem description!

# TODO: Add your physical parameters here
# EXAMPLES:
#   m = 5.0      # mass (kg)
#   k = 3.0      # spring constant (N/m)
#   b = 0.5      # damping coefficient (N*s/m)
#   ell = 0.3    # length (m)
#   g = 9.8      # gravity (m/s^2) - this is standard, don't change!

param1 = None  # REPLACE with actual parameter (with units in comment!)
param2 = None  # REPLACE with actual parameter
# ADD MORE AS NEEDED


# ═══════════════════════════════════════════════════════════════════════════════
# Chapter 3: INITIAL CONDITIONS
# ═══════════════════════════════════════════════════════════════════════════════
# Where does the system START?
# Usually set to zero unless problem specifies otherwise.

# TODO: Set initial state values
# EXAMPLES:
#   z0 = 0.0         # initial position (m)
#   zdot0 = 0.0      # initial velocity (m/s)
#   theta0 = np.radians(0.0)  # initial angle (radians!)
#   thetadot0 = 0.0  # initial angular velocity (rad/s)

state1_initial = 0.0  # RENAME and set (e.g., z0, theta0)
state2_initial = 0.0  # RENAME and set (e.g., zdot0, thetadot0)
# ADD MORE IF NEEDED

# Simulation parameters (standard values - usually don't change these)
t0 = 0.0  # start time (s)
tf = 50.0  # end time (s)
ts = 0.01  # time step (s) - smaller = more accurate but slower


# ═══════════════════════════════════════════════════════════════════════════════
# Chapter 4: EQUILIBRIUM/LINEARIZATION POINT
# ═══════════════════════════════════════════════════════════════════════════════
# Where do we linearize the system? Often at rest position.

# TODO: Calculate equilibrium state and input
# This is where xdot = 0 (system not changing)
#
# EXAMPLE (mass at rest):
#   x_eq = np.zeros(2)    # [position=0, velocity=0]
#   u_eq = np.array([0])  # no force needed to stay at rest
#
# EXAMPLE (arm hanging down):
#   x_eq = np.zeros(2)         # [theta=0, thetadot=0]
#   u_eq = np.array([m*g*ell/2])  # torque to counteract gravity

x_eq = np.zeros(2)  # CHANGE if needed
u_eq = np.array([0])  # CALCULATE equilibrium input


# ═══════════════════════════════════════════════════════════════════════════════
# Chapter 5: TRANSFER FUNCTION
# ═══════════════════════════════════════════════════════════════════════════════
# Transfer function: G(s) = num(s) / den(s) = b0 / (s^2 + a1*s + a0)
# Derive this in Chapter 5 using Laplace transforms.

# TODO: Calculate transfer function coefficients
# Get these from your Laplace transform derivation!
#
# EXAMPLE (mass-spring-damper):
#   tf_num = [1/m]                # numerator: b0
#   tf_den = [1, b/m, k/m]        # denominator: [1, a1, a0]

tf_num = [None]  # REPLACE with [b0]
tf_den = [1, None, None]  # REPLACE with [1, a1, a0]


# ═══════════════════════════════════════════════════════════════════════════════
# Chapter 6: STATE-SPACE REPRESENTATION
# ═══════════════════════════════════════════════════════════════════════════════
# State-space form: xdot = A*x + B*u,  y = C*x + D*u
# Derive these matrices in Chapter 6.

# TODO: Define state-space matrices
# Get these from your linearization in Chapter 6!
#
# EXAMPLE (2-state system):
#   A = np.array([[0, 1],
#                 [-k/m, -b/m]])
#   B = np.array([[0],
#                 [1/m]])
#   Cm = np.array([[1, 0]])  # measure position only
#   Cr = Cm                  # reference has same dimensions as output
#   D = np.zeros((1, 1))

A = np.array([[None, None], [None, None]])  # REPLACE with your A matrix
B = np.array([[None], [None]])  # REPLACE with your B matrix
Cm = None  # REPLACE - What can you measure? (measurement matrix)
Cr = Cm  # Reference has same dimensions as reference output (thing we are trying to control)
D = np.zeros((Cm.shape[0], B.shape[1]))  # Usually zero


# ═══════════════════════════════════════════════════════════════════════════════
# Chapter 8: ACTUATOR CONSTRAINTS
# ═══════════════════════════════════════════════════════════════════════════════
# Real actuators have limits! Motors can't produce infinite force/torque.

# TODO: Set maximum control input
# EXAMPLES:
#   force_max = 5.0  # maximum force (N)
#   tau_max = 1.0    # maximum torque (N*m)

input_max = None  # REPLACE with maximum control input (with units!)


# ═══════════════════════════════════════════════════════════════════════════════
# STUDENT TIPS
# ═══════════════════════════════════════════════════════════════════════════════
"""
UNITS MATTER!
- Always use SI units: meters, kilograms, seconds, radians
- NEVER use degrees - convert to radians with np.radians(deg)
- Keep track of units in comments: # mass (kg), # angle (rad), etc.

CHECKING YOUR WORK:
- Do the units work out? [force] = [mass]*[acceleration] ✓
- Are matrices the right size? A should be (n×n), B should be (n×m)
- Does x_eq make physical sense? (e.g., hanging arm has theta=0)
- Print intermediate values to debug!

COMMON ERRORS:
❌ Using degrees instead of radians
❌ Wrong matrix dimensions (rows vs columns)
❌ Forgetting to calculate u_eq (gravity compensation)
❌ Transfer function coefficients in wrong order

TESTING:
After filling this out, test with:
>>> import D_mass.params as P  # or your system name
>>> print(P.A)
>>> print(P.B)
>>> # Check that matrices look right!
"""
