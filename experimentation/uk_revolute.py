from dataclasses import dataclass, field

import numpy as np


def tilde(v: np.ndarray) -> np.ndarray:
    """
    Creates skew symmetric tilde matrix
    """
    assert len(v) == 3
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def mrp_to_dcm(sigma: np.ndarray) -> np.ndarray:
    """
    Converts a modified rodrigues parameter to a DCM
    """
    assert sigma.shape == (3, 1)
    sigma_tilde = tilde(sigma)

    return (
        np.eye(3)
        + (8 * sigma_tilde @ sigma_tilde - 4 * (1 - sigma.T @ sigma) * sigma_tilde)
        / (1 + sigma.T @ sigma) ** 2
    )


def mrp_b_matrix(sigma: np.ndarray) -> np.ndarray:
    """
    Returns the MRP B matrix $\\dot{\\sigma} = 1 / 4 * B @ \\omega$
    """
    assert sigma.shape == (3, 1)
    sigma_tilde = tilde(sigma)

    return (1 - sigma.T @ sigma) * np.eye(3) + 2 * sigma_tilde + 2 * sigma @ sigma.T


def mrp_b_dot_matrix(sigma: np.ndarray, sigma_dot: np.ndarray) -> np.ndarray:
    """
    The derivative of the B matrix
    """
    assert sigma.shape == (3, 1)
    assert sigma_dot.shape == (3, 1)
    sigma_dot_tilde = tilde(sigma_dot)

    return (
        (-2 * sigma.T @ sigma_dot) * np.eye(3)
        + 2 * sigma_dot_tilde
        + 2 * sigma_dot @ sigma.T
        + 2 * sigma @ sigma_dot.T
    )


def mrp_b_inv_matrix(sigma: np.ndarray) -> np.ndarray:
    """
    Returns the MRP B^{-1} matrix $\\omega = 4 * B^{-1} @ \\dot{\\sigma}$
    """
    assert sigma.shape == (3, 1)
    sigma_tilde = tilde(sigma)

    return (
        (1 - sigma.T @ sigma) * np.eye(3) - 2 * sigma_tilde + 2 * sigma @ sigma.T
    ) / (1 + sigma.T @ sigma) ** 2


def mrp_b_inv_dot_matrix(sigma: np.ndarray, sigma_dot: np.ndarray) -> np.ndarray:
    """
    The derivative of the B^{-1} matrix
    """
    assert sigma.shape == (3, 1)
    assert sigma_dot.shape == (3, 1)
    sigma_tilde = tilde(sigma)
    sigma_dot_tilde = tilde(sigma_dot)

    return (
        (
            (-2 * sigma.T @ sigma_dot) * np.eye(3)
            - 2 * sigma_dot_tilde
            + 2 * sigma_dot @ sigma.T
            + 2 * sigma @ sigma_dot.T
        )
        * (1 + sigma.T @ sigma)
        - 4
        * sigma.T
        @ sigma_dot
        * ((1 - sigma.T @ sigma) * np.eye(3) - 2 * sigma_tilde + 2 * sigma @ sigma.T)
    ) / (1 + sigma.T @ sigma) ** 3


@dataclass
class RigidbodyKinematics:
    """
    Derived kinematic properties of a Rigidbody used in calculation
    """

    c_0_to_body: np.ndarray = np.zeros((3, 3))
    c_body_to_0: np.ndarray = np.zeros((3, 3))
    b: np.ndarray = np.zeros((3, 3))
    b_inv: np.ndarray = np.zeros((3, 3))
    b_dot: np.ndarray = np.zeros((3, 3))
    b_inv_dot: np.ndarray = np.zeros((3, 3))
    omega_body_wrt_0_in_body: np.ndarray = np.zeros((3, 1))
    tilde_omega_body_wrt_0_in_body: np.ndarray = np.zeros((3, 3))
    tilde_omega_body_wrt_0_in_body_squared: np.ndarray = np.zeros((3, 3))


@dataclass
class Rigidbody:
    """
    A single rigidbody
    """

    m_body: float
    inertia_body_wrt_cm_in_body: np.ndarray
    r_body_wrt_0_in_0: np.ndarray
    sigma_0_to_body: np.ndarray
    v_body_wrt_0_in_0: np.ndarray
    sigma_dot_0_to_body: np.ndarray
    body_num: int
    kinematics: RigidbodyKinematics = field(
        default_factory=lambda: RigidbodyKinematics()
    )

    def __post_init__(self) -> None:
        assert self.m_body > 0
        assert self.inertia_body_wrt_cm_in_body.shape == (3, 3)
        assert self.r_body_wrt_0_in_0.shape == (3, 1)
        assert self.sigma_0_to_body.shape == (3, 1)
        assert self.v_body_wrt_0_in_0.shape == (3, 1)
        assert self.sigma_dot_0_to_body.shape == (3, 1)
        assert self.body_num >= 0

    def populate_kinematics(self) -> None:
        self.kinematics.c_0_to_body = mrp_to_dcm(self.sigma_0_to_body)
        self.kinematics.c_body_to_0 = mrp_to_dcm(-self.sigma_0_to_body)
        self.kinematics.b = mrp_b_matrix(self.sigma_0_to_body)
        self.kinematics.b_inv = mrp_b_inv_matrix(self.sigma_0_to_body)
        self.kinematics.b_dot = mrp_b_dot_matrix(
            self.sigma_0_to_body, self.sigma_dot_0_to_body
        )
        self.kinematics.b_inv_dot = mrp_b_inv_dot_matrix(
            self.sigma_0_to_body, self.sigma_dot_0_to_body
        )
        self.kinematics.omega_body_wrt_0_in_body = (
            1 / 4 * self.kinematics.b @ self.sigma_dot_0_to_body
        )
        self.kinematics.tilde_omega_body_wrt_0_in_body = tilde(
            self.kinematics.omega_body_wrt_0_in_body
        )
        self.kinematics.tilde_omega_body_wrt_0_in_body_squared = (
            self.kinematics.tilde_omega_body_wrt_0_in_body
            @ self.kinematics.tilde_omega_body_wrt_0_in_body
        )


@dataclass
class RevoluteJointKinematics:
    """
    Derived kinematic properties of a RevoluteJoint used in calculation
    """

    tilde_r_joint_wrt_1_in_1: np.ndarray = np.zeros((3, 3))
    tilde_r_joint_wrt_2_in_2: np.ndarray = np.zeros((3, 3))
    tilde_joint_direction_in_1: np.ndarray = np.zeros((3, 3))
    tilde_joint_direction_in_2: np.ndarray = np.zeros((3, 3))
    body1_joint_direction_in_0: np.ndarray = np.zeros((3, 1))
    body2_joint_direction_in_0: np.ndarray = np.zeros((3, 1))
    tilde_body1_joint_direction_in_0: np.ndarray = np.zeros((3, 3))
    tilde_body2_joint_direction_in_0: np.ndarray = np.zeros((3, 3))
    body1_joint_direction_dot_in_0: np.ndarray = np.zeros((3, 1))
    body2_joint_direction_dot_in_0: np.ndarray = np.zeros((3, 1))


@dataclass
class RevoluteJoint:
    """
    A revolute joint joining two rigidbodies
    """

    body1_idx: int
    body2_idx: int
    r_joint_wrt_1_in_1: np.ndarray
    r_joint_wrt_2_in_2: np.ndarray
    joint_direction_in_1: np.ndarray
    joint_direction_in_2: np.ndarray
    joint_num: int
    kinematics: RevoluteJointKinematics = field(
        default_factory=lambda: RevoluteJointKinematics()
    )

    def __post_init__(self) -> None:
        assert self.body1_idx >= 0
        assert self.body2_idx >= 0
        assert self.body1_idx != self.body2_idx
        assert self.r_joint_wrt_1_in_1.shape == (3, 1)
        assert self.r_joint_wrt_2_in_2.shape == (3, 1)
        assert self.joint_direction_in_1.shape == (3, 1)
        assert self.joint_direction_in_2.shape == (3, 1)
        assert self.joint_num >= 0

    def populate_kinematics(self, body1: Rigidbody, body2: Rigidbody) -> None:
        self.kinematics.tilde_r_joint_wrt_1_in_1 = tilde(self.r_joint_wrt_1_in_1)
        self.kinematics.tilde_r_joint_wrt_2_in_2 = tilde(self.r_joint_wrt_2_in_2)
        self.kinematics.tilde_joint_direction_in_1 = tilde(self.joint_direction_in_1)
        self.kinematics.tilde_joint_direction_in_2 = tilde(self.joint_direction_in_2)
        self.kinematics.body1_joint_direction_in_0 = body1.kinematics.c_body_to_0
        self.kinematics.body2_joint_direction_in_0 = body2.kinematics.c_body_to_0
        self.kinematics.tilde_body1_joint_direction_in_0 = tilde(
            self.kinematics.body1_joint_direction_in_0
        )
        self.kinematics.tilde_body2_joint_direction_in_0 = tilde(
            self.kinematics.body2_joint_direction_in_0
        )
        self.kinematics.body1_joint_direction_dot_in_0 = (
            body1.kinematics.c_body_to_0
            @ body1.kinematics.tilde_omega_body_wrt_0_in_body
            @ self.joint_direction_in_1
        )
        self.kinematics.body2_joint_direction_dot_in_0 = (
            body2.kinematics.c_body_to_0
            @ body2.kinematics.tilde_omega_body_wrt_0_in_body
            @ self.joint_direction_in_2
        )


@dataclass
class MultiRigidbody:
    """
    A collection of bodies joined by joints
    """

    bodies: list[Rigidbody] = field(default_factory=list)
    joints: list[RevoluteJoint] = field(default_factory=list)

    def add_body(
        self,
        m_body: float,
        inertia_body_wrt_cm_in_body: np.ndarray,
        r_body_wrt_0_in_0: np.ndarray = np.zeros((3, 1)),
        sigma_0_to_body: np.ndarray = np.zeros((3, 1)),
        v_body_wrt_0_in_0: np.ndarray = np.zeros((3, 1)),
        sigma_dot_0_to_body: np.ndarray = np.zeros((3, 1)),
    ) -> None:
        body_num = len(self.bodies)
        self.bodies.append(
            Rigidbody(
                m_body,
                inertia_body_wrt_cm_in_body,
                r_body_wrt_0_in_0,
                sigma_0_to_body,
                v_body_wrt_0_in_0,
                sigma_dot_0_to_body,
                body_num,
            )
        )

    def add_joint(
        self,
        body1_idx: int,
        body2_idx: int,
        r_joint_wrt_1_in_1: np.ndarray = np.zeros((3, 1)),
        r_joint_wrt_2_in_2: np.ndarray = np.zeros((3, 1)),
        joint_direction_in_1: np.ndarray = np.array([[0], [0], [1]]),
        joint_direction_in_2: np.ndarray = np.array([[0], [0], [1]]),
    ) -> None:
        assert body1_idx >= 0
        assert body1_idx < len(self.bodies)
        assert body2_idx >= 0
        assert body2_idx < len(self.bodies)
        assert body1_idx != body2_idx
        joint_num = len(self.joints)
        self.joints.append(
            RevoluteJoint(
                body1_idx,
                body2_idx,
                r_joint_wrt_1_in_1,
                r_joint_wrt_2_in_2,
                joint_direction_in_1,
                joint_direction_in_2,
                joint_num,
            )
        )

    def state(self) -> np.ndarray:
        body_states: list[np.ndarray] = []
        for body in self.bodies:
            body_states.append(body.r_body_wrt_0_in_0)
            body_states.append(body.sigma_0_to_body)

        return np.stack(body_states)

    def state_dot(self) -> np.ndarray:
        body_state_dots: list[np.ndarray] = []
        for body in self.bodies:
            body_state_dots.append(body.v_body_wrt_0_in_0)
            body_state_dots.append(body.sigma_dot_0_to_body)

        return np.stack(body_state_dots)

    def populate_kinematics(self) -> None:
        for body in self.bodies:
            body.populate_kinematics()

        for joint in self.joints:
            joint.populate_kinematics(
                self.bodies[joint.body1_idx], self.bodies[joint.body2_idx]
            )

    def uk_a_matrix_and_b_vector(self) -> tuple[np.ndarray, np.ndarray]:
        state_dim = len(self.bodies) * 6
        n_constraint_eqns = len(self.joints) * 6

        a_matrix = np.zeros((n_constraint_eqns, state_dim))
        b_vector = np.zeros((n_constraint_eqns, 1))

        for joint_idx, joint in enumerate(self.joints):
            # Get bodies
            body1 = self.bodies[joint.body1_idx]
            body2 = self.bodies[joint.body2_idx]

            # Define slices to index into A and b
            position_constraint_slice = slice(6 * joint_idx, 6 * joint_idx + 3)
            rotation_constraint_slice = slice(6 * joint_idx + 3, 6 * joint_idx + 6)
            body1_position_slice = slice(6 * joint.body1_idx, 6 * joint.body1_idx + 3)
            body1_mrp_slice = slice(6 * joint.body1_idx + 3, 6 * joint.body1_idx + 6)
            body2_position_slice = slice(6 * joint.body2_idx, 6 * joint.body2_idx + 3)
            body2_mrp_slice = slice(6 * joint.body2_idx + 3, 6 * joint.body2_idx + 6)

            # Populate A matrix
            a_matrix[position_constraint_slice, body1_position_slice] = np.eye(3)
            a_matrix[position_constraint_slice, body2_position_slice] = -np.eye(3)
            a_matrix[position_constraint_slice, body1_mrp_slice] = (
                -4
                * body1.kinematics.c_body_to_0
                @ joint.kinematics.tilde_r_joint_wrt_1_in_1
                @ body1.kinematics.b_inv
            )
            a_matrix[position_constraint_slice, body2_mrp_slice] = (
                4
                * body2.kinematics.c_body_to_0
                @ joint.kinematics.tilde_r_joint_wrt_2_in_2
                @ body2.kinematics.b_inv
            )
            a_matrix[rotation_constraint_slice, body1_mrp_slice] = (
                4
                * joint.kinematics.tilde_body2_joint_direction_in_0
                @ body1.kinematics.c_body_to_0
                @ joint.kinematics.tilde_joint_direction_in_1
                @ body1.kinematics.b_inv
            )
            a_matrix[rotation_constraint_slice, body2_mrp_slice] = (
                -4
                * joint.kinematics.tilde_body1_joint_direction_in_0
                @ body2.kinematics.c_body_to_0
                @ joint.kinematics.tilde_joint_direction_in_2
                @ body2.kinematics.b_inv
            )

            # Populate b vector
            b_vector[position_constraint_slice, 0] = (
                4
                * body1.kinematics.c_body_to_0
                @ joint.kinematics.tilde_r_joint_wrt_1_in_1
                @ body1.kinematics.b_inv_dot
                @ body1.sigma_dot_0_to_body
                - body1.kinematics.c_body_to_0
                @ body1.kinematics.tilde_omega_body_wrt_0_in_body_squared
                @ joint.r_joint_wrt_1_in_1
                - 4
                * body2.kinematics.c_body_to_0
                @ joint.kinematics.tilde_r_joint_wrt_2_in_2
                @ body2.kinematics.b_inv_dot
                @ body2.sigma_dot_0_to_body
                + body2.kinematics.c_body_to_0
                @ body2.kinematics.tilde_omega_body_wrt_0_in_body_squared
                @ joint.r_joint_wrt_2_in_2
            )
            b_vector[rotation_constraint_slice, 0] = (
                -4
                * joint.kinematics.tilde_body2_joint_direction_in_0
                @ body1.kinematics.c_body_to_0
                @ joint.kinematics.tilde_joint_direction_in_1
                @ body1.kinematics.b_inv_dot
                @ body1.sigma_dot_0_to_body
                + joint.kinematics.tilde_body2_joint_direction_in_0
                @ body1.kinematics.c_body_to_0
                @ body1.kinematics.tilde_omega_body_wrt_0_in_body
                @ joint.joint_direction_in_1
                - 2
                * tilde(joint.kinematics.body1_joint_direction_dot_in_0)
                @ joint.kinematics.body2_joint_direction_dot_in_0
                + 4
                * joint.kinematics.tilde_body1_joint_direction_in_0
                @ body2.kinematics.c_body_to_0
                @ joint.kinematics.tilde_joint_direction_in_2
                @ body2.kinematics.b_inv_dot
                @ body2.sigma_dot_0_to_body
                - joint.kinematics.tilde_body1_joint_direction_in_0
                @ body2.kinematics.c_body_to_0
                @ body2.kinematics.tilde_omega_body_wrt_0_in_body
                @ joint.joint_direction_in_2
            )

        return a_matrix, b_vector

    def uk_dynamics(self) -> np.ndarray:
        self.populate_kinematics()
        unconstrained_state_ddot = self.unconstrained_dynamics()
