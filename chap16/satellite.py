# 3rd-party
import control as cnt
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import C_satellite


P = C_satellite.params

use_dB = True
plant_inner = cnt.tf(P.tf_inner_num, P.tf_inner_den)
plant_outer = cnt.tf(P.tf_outer_num, P.tf_outer_den)

pid = C_satellite.ControllerPID()
kp_theta, kd_theta, sigma = pid.kp_theta, pid.kd_theta, pid.sigma
controller_inner = cnt.tf([kd_theta + kp_theta * sigma, kp_theta], [sigma, 1])

kp_phi, ki_phi, kd_phi = pid.kp_phi, pid.ki_phi, pid.kd_phi
controller_outer = cnt.tf(
    [kd_phi + kp_phi * sigma, kp_phi + ki_phi * sigma, ki_phi], [sigma, 1, 0]
)

# open-loop system
sys_inner_ol = cnt.series(controller_inner, plant_inner)
sys_outer_ol = cnt.series(controller_outer, plant_outer)

# reference
A = 20.0
ref = cnt.tf([1], [1, 0, 0])


if __name__ == "__main__":
    cnt.bode(
        [plant_inner, sys_inner_ol, ref],
        dB=use_dB,
        omega_limits=(1e-2, 1e3),
        label=["P(s)", r"$C_{pd}(s)P(s)$", r"$1/s^2$"],
        legend_loc="best",
        title="HW 16 Bode Plots: Satellite - Inner Loop",
    )

    # part a)
    omega_r = 0.01
    mag_CP, _, _ = cnt.frequency_response(sys_inner_ol, omega=[omega_r])
    mag_ref, _, _ = cnt.frequency_response(ref, omega=[omega_r])
    Ma = mag_CP[0] / mag_ref[0]
    max_error = A / Ma
    print(f"Max error to reference is {max_error:.2f}")

    # part b)
    omega_d_in = 0.1
    mag_CP, _, _ = cnt.frequency_response(sys_inner_ol, omega=[omega_d_in])
    mag_P, _, _ = cnt.frequency_response(plant_inner, omega=[omega_d_in])
    gamma_d_in = mag_P[0] / (1 + mag_CP[0])
    print(f"{gamma_d_in:.2%} of disturbance is passed to output")

    plt.show()

    cnt.bode(
        [plant_outer, sys_outer_ol],
        dB=use_dB,
        omega_limits=(1e-2, 1e3),
        label=["P(s)", r"$C_{pid}(s)P(s)$"],
        legend_loc="best",
        title="HW 16 Bode Plots: Satellite - Outer Loop",
    )

    # part c)
    omega_n = 10.0
    mag_CP, _, _ = cnt.frequency_response(sys_outer_ol, omega=[omega_n])
    gamma_n = mag_CP[0] / (1 + mag_CP[0])
    print(f"{gamma_n:.2%} of noise is passed to output")

    plt.show()
