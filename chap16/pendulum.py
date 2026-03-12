# 3rd-party
import control as cnt
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import B_pendulum


P = B_pendulum.params

use_dB = True
plant_inner = cnt.tf(P.tf_inner_num, P.tf_inner_den)
plant_outer = cnt.tf(P.tf_outer_num, P.tf_outer_den)

pid = B_pendulum.ControllerPID()
kp_theta, kd_theta, sigma = pid.kp_theta, pid.kd_theta, pid.sigma
controller_inner = cnt.tf([kd_theta + kp_theta * sigma, kp_theta], [sigma, 1])

kp_z, ki_z, kd_z = pid.kp_z, pid.ki_z, pid.kd_z
controller_outer = cnt.tf(
    [kd_z + kp_z * sigma, kp_z + ki_z * sigma, ki_z], [sigma, 1, 0]
)

# open-loop system
sys_inner_ol = cnt.series(controller_inner, plant_inner)
sys_outer_ol = cnt.series(controller_outer, plant_outer)


if __name__ == "__main__":
    cnt.bode(
        [plant_inner, sys_inner_ol],
        dB=use_dB,
        omega_limits=(1e-2, 1e3),
        label=["P(s)", r"$C_{pd}(s)P(s)$"],
        legend_loc="best",
        title="HW 16 Bode Plots: Pendulum on a Cart - Inner Loop",
    )

    # part a)
    omega_r = 1.0
    mag_CP, _, _ = cnt.frequency_response(sys_inner_ol, omega=[omega_r])
    gamma_r = 1 / (1 + mag_CP[0])
    print(f"Tracks to {gamma_r:.2%} error")

    # part b)
    omega_n = 200.0
    mag_CP, _, _ = cnt.frequency_response(sys_inner_ol, omega=[omega_n])
    gamma_n = mag_CP[0] / (1 + mag_CP[0])
    print(f"{gamma_n:.2%} of noise is passed to output")

    plt.show()

    cnt.bode(
        [plant_outer, sys_outer_ol],
        dB=use_dB,
        omega_limits=(1e-2, 1e3),
        label=["P(s)", r"$C_{pid}(s)P(s)$"],
        legend_loc="best",
        title="HW 16 Bode Plots: Pendulum on a Cart - Outer Loop",
    )

    # part c)
    A = 50.0
    omega_r = 0.01
    mag_CP, _, _ = cnt.frequency_response(sys_outer_ol, omega=[omega_r])
    gamma_r = 1 / (1 + mag_CP[0])
    max_error = gamma_r * A
    print(f"{gamma_r = :.2e}")
    print(f"Max error to reference is {max_error:.2e}")

    plt.show()
