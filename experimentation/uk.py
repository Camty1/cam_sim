"""
Using UK dynamics to simulate simple systems
"""

import matplotlib

matplotlib.use("QTAgg")
import matplotlib.pyplot as plt
import numpy as np

# Two point masses M_1 and M_2 are a distance R away from each other.
#
# Unconstrained EOM:
#
# M_1 \ddot{r_1} = f_1
# M_2 \ddot{r_2} = f_2
#
# m_matrix = diag(M_1, M_1, M_1, M_2, M_2, M_2)
#
# Constraint equation:
#
# |r_2 - r_1|^2 = R^2
# Let d = r_2 - r_1.  Take second derivative of both sides:
# d/dt(d*d) = d/dt(R^2)
# d * \dot{d} = 0
# d/dt(d * \dot{d}) = 0
# d * \ddot{d} = -\dot{d}^2
#
# Unsubstitude d for derivative quantites
#
# [d, -d] * [r_1, r_2]^T = |v_2 - v_1|^2

DT = 0.01
TF = 1000

M_1 = 1
M_2 = 1
R = 1

INV_SQRT_M_1 = 1 / np.sqrt(M_1)
INV_SQRT_M_2 = 1 / np.sqrt(M_2)


def uk_2mass(q: np.ndarray, q_dot: np.ndarray, f: np.ndarray) -> np.ndarray:
    """
    Returns q_ddot
    """
    r_1, r_2 = q[:3], q[3:]
    v_1, v_2 = q_dot[:3], q_dot[3:]
    f_1, f_2 = f[:3], f[3:]

    a_1_u, a_2_u = f_1 / M_1, f_2 / M_2
    q_u_ddot = np.concatenate([a_1_u, a_2_u])

    d = r_2 - r_1
    d_dot = v_2 - v_1

    a_matrix = np.concatenate([d, -d]).reshape((1, -1))
    b = np.dot(d_dot, d_dot)

    m_inv_sqrt = np.diag([INV_SQRT_M_1] * 3 + [INV_SQRT_M_2] * 3)
    q_ddot = q_u_ddot + m_inv_sqrt @ np.linalg.pinv(a_matrix @ m_inv_sqrt) @ (
        b - a_matrix @ q_u_ddot
    )

    return q_ddot


def main() -> None:
    num_steps = int(TF / DT)
    q = np.zeros((num_steps + 1, 6))
    q[0, :] = np.array([0, 0, 0, R, 0, 0])
    q_dot = np.zeros((num_steps + 1, 6))
    for i in range(num_steps):
        q[i + 1, :] = q[i, :] + DT * q_dot[i, :]
        q_ddot = uk_2mass(
            q[i, :],
            q_dot[i, :],
            np.array([0.0, 1.0, 0.0, 0.0, -1.0, 1.0]) if i == 0 else np.zeros(6),
        )
        q_dot[i + 1, :] = q_dot[i, :] + DT * q_ddot

    ax = plt.figure().add_subplot(projection="3d")
    ax.plot(q[:, 0], q[:, 1], q[:, 2])
    ax.plot(q[:, 3], q[:, 4], q[:, 5])
    plt.show()


if __name__ == "__main__":
    main()
