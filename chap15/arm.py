# 3rd-party
import control as cnt
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies.A_arm import params as P


use_dB = False
plant = cnt.tf(P.tf_num, P.tf_den)


if __name__ == "__main__":
    cnt.bode(
        plant,
        dB=use_dB,
        label="P(s)",
        legend_loc="best",
        title="HW 15 Bode Plots: Robot Arm",
    )

    plt.show()
