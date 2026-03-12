import numpy as np

# g = 
# ell1 = 
# ell2 = 
# ell3x = 
# ell3y = 
# ell3z = 
# ellT = 
# d = 
# m1 = 
# J1x = 
# J1y = 
# J1z = 
# m2 = 
# J2x = 
# J2y = 
# J2z = 
# m3 = 
# J3x = 
# J3y = 
# J3z = 


##### Chapter 4
# mixing matrices (see end of Chapter 4 in lab manual)
# mixing is a UAV term for taking body forces/torques to individual motor forces
unmixer = np.array([[1.0, 1.0], [d, -d]])  # [F, tau] = unmixer @ [fl, fr]
mixer = np.linalg.inv(unmixer)  # [fl, fr] = mixer @ [F, tau]

