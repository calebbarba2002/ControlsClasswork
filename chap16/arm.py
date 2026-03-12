# 3rd-party
import control as cnt
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import A_arm


P = A_arm.params

use_dB = True
plant = cnt.tf(P.tf_num, P.tf_den)

pid = A_arm.ControllerPID()
kp, ki, kd, sigma = pid.kp, pid.ki, pid.kd, pid.sigma
controller = cnt.tf([kd + kp * sigma, kp + ki * sigma, ki], [sigma, 1, 0])

# open-loop system
sys_ol = cnt.series(controller, plant)

# reference signal
A = 5.0
ref = cnt.tf([1], [1, 0, 0])

if __name__ == "__main__":
    cnt.bode(
        [plant, sys_ol, ref],
        dB=use_dB,
        omega_limits=(1e-2, 1e3),
        label=["P(s)", r"$C_{pid}(s)P(s)$", r"$1/s^2$"],
        legend_loc="best",
        title="HW 16 Bode Plots: Robot Arm",
    )

    # part a)
    omega_r = 0.4
    mag_CP, _, _ = cnt.frequency_response(sys_ol, omega=[omega_r])
    gamma_r = 1 / (1 + mag_CP[0])
    print(f"Tracks to {gamma_r:.2%} error")

    # part b)
    omega_r = 0.02  # choose something small
    mag_CP, _, _ = cnt.frequency_response(sys_ol, omega=[omega_r])
    mag_ref, _, _ = cnt.frequency_response(ref, omega=[omega_r])
    Ma = mag_CP[0] / mag_ref[0]
    max_error = A / Ma
    print(f"Max error to reference is {max_error:.2f}")

    # part c)
    omega_d_in = 0.01
    mag_CP, _, _ = cnt.frequency_response(sys_ol, omega=[omega_d_in])
    mag_P, _, _ = cnt.frequency_response(plant, omega=[omega_d_in])
    gamma_d_in = mag_P[0] / (1 + mag_CP)[0]
    print(f"{gamma_d_in:.2%} of input disturbance is passed to output")

    # part d)
    omega_n = 100.0
    mag_CP, _, _ = cnt.frequency_response(sys_ol, omega=[omega_n])
    gamma_n = mag_CP[0] / (1 + mag_CP[0])
    print(f"{gamma_n:.2%} of noise is passed to output")

    plt.show()
