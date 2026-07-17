from dataclasses import dataclass, field

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

np.set_printoptions(threshold=np.inf, linewidth=250)


def tilde(v: np.ndarray) -> np.ndarray:
    """
    Creates skew symmetric tilde matrix
    """
    assert len(v) == 3

    return np.array(
        [
            [0, -v[2].item(), v[1].item()],
            [v[2].item(), 0, -v[0].item()],
            [-v[1].item(), v[0].item(), 0],
        ]
    )


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

    c_0_to_body: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    c_body_to_0: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    b: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    b_inv: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    b_dot: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    b_inv_dot: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    omega_body_wrt_0_in_body: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 1))
    )
    tilde_omega_body_wrt_0_in_body: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )
    tilde_omega_body_wrt_0_in_body_squared: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )


@dataclass
class RigidbodyDerivedMassProperties:
    """
    Mass properties that do not change over time
    """

    initialized: bool = False
    inertia_inv_body_wrt_cm_in_body: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )


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
    derived_mass_properties: RigidbodyDerivedMassProperties = field(
        default_factory=lambda: RigidbodyDerivedMassProperties()
    )
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

    def initialize_derived_mass_properties(self) -> None:
        """
        Initialize derived mass properties (should be called once at the beginning of the function)
        """
        assert not self.derived_mass_properties.initialized

        self.derived_mass_properties.inertia_inv_body_wrt_cm_in_body = np.linalg.inv(
            self.inertia_body_wrt_cm_in_body
        )

        self.derived_mass_properties.initialized = True

    def populate_kinematics(self) -> None:
        """
        Populates the kinematics (should be called each timestep)
        """
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
            4 * self.kinematics.b_inv @ self.sigma_dot_0_to_body
        )
        self.kinematics.tilde_omega_body_wrt_0_in_body = tilde(
            self.kinematics.omega_body_wrt_0_in_body
        )
        self.kinematics.tilde_omega_body_wrt_0_in_body_squared = (
            self.kinematics.tilde_omega_body_wrt_0_in_body
            @ self.kinematics.tilde_omega_body_wrt_0_in_body
        )

    def unconstrained_dynamics(
        self,
        force_body_wrt_0_in_0: np.ndarray = np.zeros((3, 1)),
        moment_body_wrt_0_in_body: np.ndarray = np.zeros((3, 1)),
    ) -> np.ndarray:
        """
        Calculates the unconstrained dynamics of the rigidbody
        """
        assert self.derived_mass_properties.initialized
        assert force_body_wrt_0_in_0.shape == (3, 1)
        assert moment_body_wrt_0_in_body.shape == (3, 1)

        scaled_r_ddot_body_wrt_0_in_0 = force_body_wrt_0_in_0
        scaled_sigma_ddot_0_to_body = (
            self.kinematics.b_dot @ self.kinematics.omega_body_wrt_0_in_body
            + self.kinematics.b
            @ (
                self.derived_mass_properties.inertia_inv_body_wrt_cm_in_body
                @ (
                    moment_body_wrt_0_in_body
                    - self.kinematics.tilde_omega_body_wrt_0_in_body
                    @ self.inertia_body_wrt_cm_in_body
                    @ self.kinematics.omega_body_wrt_0_in_body
                )
            )
        )

        return np.concatenate(
            [scaled_r_ddot_body_wrt_0_in_0, scaled_sigma_ddot_0_to_body], axis=0
        )


@dataclass
class RevoluteJointKinematics:
    """
    Derived kinematic properties of a RevoluteJoint used in calculation
    """

    tilde_r_joint_wrt_1_in_1: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )
    tilde_r_joint_wrt_2_in_2: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )
    tilde_joint_direction_in_1: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )
    tilde_joint_direction_in_2: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )
    body1_joint_direction_in_0: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 1))
    )
    body2_joint_direction_in_0: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 1))
    )
    tilde_body1_joint_direction_in_0: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )
    tilde_body2_joint_direction_in_0: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )
    body1_joint_direction_dot_in_0: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 1))
    )
    body2_joint_direction_dot_in_0: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 1))
    )


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
        """
        Populates the kinematics of the joint (called at the start of each timestep)
        """
        self.kinematics.tilde_r_joint_wrt_1_in_1 = tilde(self.r_joint_wrt_1_in_1)
        self.kinematics.tilde_r_joint_wrt_2_in_2 = tilde(self.r_joint_wrt_2_in_2)
        self.kinematics.tilde_joint_direction_in_1 = tilde(self.joint_direction_in_1)
        self.kinematics.tilde_joint_direction_in_2 = tilde(self.joint_direction_in_2)
        self.kinematics.body1_joint_direction_in_0 = (
            body1.kinematics.c_body_to_0 @ self.joint_direction_in_1
        )
        self.kinematics.body2_joint_direction_in_0 = (
            body2.kinematics.c_body_to_0 @ self.joint_direction_in_2
        )
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
class ForceGenerator:
    """
    Contains all potential forces and torques that one can apply to a given rigid body
    """

    body_idx: int
    f_wrt_0_in_0: np.ndarray
    f_wrt_0_in_body: np.ndarray
    r_f_wrt_body_in_body: np.ndarray
    tau_body_wrt_0_in_body: np.ndarray

    def __post_init__(self) -> None:
        assert self.body_idx >= 0
        assert self.f_wrt_0_in_0.shape == (3, 1)
        assert self.f_wrt_0_in_body.shape == (3, 1)
        assert self.r_f_wrt_body_in_body.shape == (3, 1)
        assert self.tau_body_wrt_0_in_body.shape == (3, 1)

    def compute_forces(self, body: Rigidbody) -> np.ndarray:
        """
        Computes the forces and moments to be applied to the body
        """
        f_total_wrt_0_in_0 = (
            self.f_wrt_0_in_0 + body.kinematics.c_body_to_0 @ self.f_wrt_0_in_body
        )
        tau_body_wrt_0 = (
            self.tau_body_wrt_0_in_body
            + tilde(self.r_f_wrt_body_in_body) @ self.f_wrt_0_in_body
        )

        return np.concatenate([f_total_wrt_0_in_0, tau_body_wrt_0], axis=0)


@dataclass
class MultiRigidbodyDerivedMassProperties:
    """
    Derived mass properties for a MultiRigidbody.  Must be initialized after adding all bodies
    """

    initialized: bool = False
    mass_matrix: np.ndarray = field(default_factory=lambda: np.zeros(0))
    mass_matrix_inv: np.ndarray = field(default_factory=lambda: np.zeros(0))
    mass_matrix_sqrt: np.ndarray = field(default_factory=lambda: np.zeros(0))
    mass_matrix_inv_sqrt: np.ndarray = field(default_factory=lambda: np.zeros(0))


@dataclass
class MultiRigidbody:
    """
    A collection of bodies joined by joints
    """

    bodies: list[Rigidbody] = field(default_factory=list)
    joints: list[RevoluteJoint] = field(default_factory=list)
    forces: list[ForceGenerator] = field(default_factory=list)
    derived_mass_properties: MultiRigidbodyDerivedMassProperties = field(
        default_factory=lambda: MultiRigidbodyDerivedMassProperties()
    )

    def add_body(
        self,
        m_body: float,
        inertia_body_wrt_cm_in_body: np.ndarray,
        r_body_wrt_0_in_0: np.ndarray = np.zeros((3, 1)),
        sigma_0_to_body: np.ndarray = np.zeros((3, 1)),
        v_body_wrt_0_in_0: np.ndarray = np.zeros((3, 1)),
        sigma_dot_0_to_body: np.ndarray = np.zeros((3, 1)),
    ) -> None:
        """
        Adds a rigidbody
        """
        assert not self.derived_mass_properties.initialized
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

    def add_revolute_joint(
        self,
        body1_idx: int,
        body2_idx: int,
        r_joint_wrt_1_in_1: np.ndarray = np.zeros((3, 1)),
        r_joint_wrt_2_in_2: np.ndarray = np.zeros((3, 1)),
        joint_direction_in_1: np.ndarray = np.array([[0], [0], [1]]),
        joint_direction_in_2: np.ndarray = np.array([[0], [0], [1]]),
    ) -> None:
        """
        Adds a revolute joint
        """
        assert body1_idx >= 0
        assert body1_idx < len(self.bodies)
        assert body2_idx >= 0
        assert body2_idx < len(self.bodies)
        assert body1_idx != body2_idx
        assert not self.derived_mass_properties.initialized
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

    def initialize_derived_mass_properties(self) -> None:
        """
        Initializes derived mass properties which are used in dynamics calculations
        """
        assert not self.derived_mass_properties.initialized
        self.derived_mass_properties.mass_matrix = np.zeros(
            (6 * len(self.bodies), 6 * len(self.bodies))
        )
        self.derived_mass_properties.mass_matrix_inv = np.zeros(
            (6 * len(self.bodies), 6 * len(self.bodies))
        )
        self.derived_mass_properties.mass_matrix_sqrt = np.zeros(
            (6 * len(self.bodies), 6 * len(self.bodies))
        )
        self.derived_mass_properties.mass_matrix_inv_sqrt = np.zeros(
            (6 * len(self.bodies), 6 * len(self.bodies))
        )
        for body_idx, body in enumerate(self.bodies):
            body.initialize_derived_mass_properties()
            start_idx = 6 * body_idx
            end_idx = 6 * (body_idx + 1)
            self.derived_mass_properties.mass_matrix[
                start_idx:end_idx, start_idx:end_idx
            ] = np.diag([body.m_body, body.m_body, body.m_body, 4, 4, 4])
            self.derived_mass_properties.mass_matrix_inv[
                start_idx:end_idx, start_idx:end_idx
            ] = np.diag(
                [1 / body.m_body, 1 / body.m_body, 1 / body.m_body, 1 / 4, 1 / 4, 1 / 4]
            )
            self.derived_mass_properties.mass_matrix_sqrt[
                start_idx:end_idx, start_idx:end_idx
            ] = np.sqrt(np.diag([body.m_body, body.m_body, body.m_body, 4, 4, 4]))
            self.derived_mass_properties.mass_matrix_inv_sqrt[
                start_idx:end_idx, start_idx:end_idx
            ] = np.sqrt(
                np.diag(
                    [
                        1 / body.m_body,
                        1 / body.m_body,
                        1 / body.m_body,
                        1 / 4,
                        1 / 4,
                        1 / 4,
                    ]
                )
            )

        self.derived_mass_properties.initialized = True

    @property
    def state(self) -> np.ndarray:
        """
        Returns x
        """
        body_states: list[np.ndarray] = []
        for body in self.bodies:
            body_states.append(body.r_body_wrt_0_in_0)
            body_states.append(body.sigma_0_to_body)

        return np.concatenate(body_states)

    @state.setter
    def state(self, val: np.ndarray) -> None:
        assert val.shape == (len(self.bodies) * 6, 1)
        for body_idx, body in enumerate(self.bodies):
            body.r_body_wrt_0_in_0 = val[body_idx * 6 : body_idx * 6 + 3]
            body.sigma_0_to_body = val[body_idx * 6 + 3 : body_idx * 6 + 6]

    @property
    def state_dot(self) -> np.ndarray:
        """
        Returns x_dot
        """
        body_state_dots: list[np.ndarray] = []
        for body in self.bodies:
            body_state_dots.append(body.v_body_wrt_0_in_0)
            body_state_dots.append(body.sigma_dot_0_to_body)

        return np.concatenate(body_state_dots)

    @state_dot.setter
    def state_dot(self, val: np.ndarray) -> None:
        assert val.shape == (len(self.bodies) * 6, 1)
        for body_idx, body in enumerate(self.bodies):
            body.v_body_wrt_0_in_0 = val[body_idx * 6 : body_idx * 6 + 3]
            body.sigma_dot_0_to_body = val[body_idx * 6 + 3 : body_idx * 6 + 6]

    def populate_kinematics(self) -> None:
        """
        Called at the beginning of each timestep, populates the derived kinematics
        """
        for body in self.bodies:
            body.populate_kinematics()

        for joint in self.joints:
            joint.populate_kinematics(
                self.bodies[joint.body1_idx], self.bodies[joint.body2_idx]
            )

    def calculate_forces(self) -> np.ndarray:
        """
        Loops through force generators to get forces and torques applied to bodies
        """
        f = np.zeros((6 * len(self.bodies), 1))
        for force in self.forces:
            body_idx = force.body_idx
            body_slice = slice(6 * body_idx, 6 * (body_idx + 1))
            f[body_slice, 0] += force.compute_forces(self.bodies[body_idx])

        return f

    def unconstrained_dynamics(self, total_force: np.ndarray) -> np.ndarray:
        """
        Loops through bodies to get M @ x_ddot
        """
        body_scaled_state_ddots: list[np.ndarray] = []
        for body_idx, body in enumerate(self.bodies):
            force_slice = slice(6 * body_idx, 6 * body_idx + 3)
            moment_slice = slice(6 * body_idx + 3, 6 * (body_idx + 1))
            f_body_wrt_0_in_0 = total_force[force_slice, 0]
            tau_body_wrt_0_in_body = total_force[moment_slice, 0]
            body_scaled_state_ddots.append(
                body.unconstrained_dynamics(f_body_wrt_0_in_0, tau_body_wrt_0_in_body)
            )

        return np.concatenate(body_scaled_state_ddots, axis=0)

    def uk_a_matrix_b_vector_with_baumgarte(
        self,
        alpha: float = 10,
        beta: float = 10,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Gets the UK A matrix and b vector from looping through constraints
        """
        state_dim = len(self.bodies) * 6
        n_constraint_eqns = len(self.joints) * 6

        a_matrix = np.zeros((n_constraint_eqns, state_dim))
        b_vector = np.zeros((n_constraint_eqns, 1))
        phi_vector = np.zeros((n_constraint_eqns, 1))
        phi_dot_vector = np.zeros((n_constraint_eqns, 1))

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
            ).flatten()
            b_vector[rotation_constraint_slice, 0] = (
                -4
                * joint.kinematics.tilde_body2_joint_direction_in_0
                @ body1.kinematics.c_body_to_0
                @ joint.kinematics.tilde_joint_direction_in_1
                @ body1.kinematics.b_inv_dot
                @ body1.sigma_dot_0_to_body
                + joint.kinematics.tilde_body2_joint_direction_in_0
                @ body1.kinematics.c_body_to_0
                @ body1.kinematics.tilde_omega_body_wrt_0_in_body_squared
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
                @ body2.kinematics.tilde_omega_body_wrt_0_in_body_squared
                @ joint.joint_direction_in_2
            ).flatten()

            phi_vector[position_constraint_slice, 0] = (
                body1.r_body_wrt_0_in_0
                + body1.kinematics.c_body_to_0 @ joint.r_joint_wrt_1_in_1
                - body2.r_body_wrt_0_in_0
                - body2.kinematics.c_body_to_0 @ joint.r_joint_wrt_2_in_2
            ).flatten()
            phi_vector[rotation_constraint_slice, 0] = (
                joint.kinematics.tilde_body1_joint_direction_in_0
                @ joint.kinematics.body2_joint_direction_in_0
            ).flatten()

        phi_dot_vector = a_matrix @ self.state_dot

        # Apply baumgarte stabilization to b_vector
        b_vector = b_vector - 2 * alpha * phi_dot_vector - beta**2 * phi_vector

        return a_matrix, b_vector

    def uk_dynamics(self) -> np.ndarray:
        """
        Gets x_ddot taking into account the unconstrained equations of motion + constraints

        x_ddot = M^{-1}q + M^{-1} A^T (A M^{-1} A^T)^{-1} (b - A M^{-1} q)
        """
        if not self.derived_mass_properties.initialized:
            raise ValueError("Derived mass properties have not been initialized")
        self.populate_kinematics()
        f = self.calculate_forces()
        q = self.unconstrained_dynamics(f)
        a_matrix, b_vector = self.uk_a_matrix_b_vector_with_baumgarte()

        # Precompute common terms
        mass_matrix_inv_q = self.derived_mass_properties.mass_matrix_inv @ q
        a_matrix_mass_matrix_inv = (
            a_matrix @ self.derived_mass_properties.mass_matrix_inv
        )

        # Solve UK dynamics
        lambda_vec = np.linalg.lstsq(
            a_matrix_mass_matrix_inv @ a_matrix.T,
            b_vector - a_matrix_mass_matrix_inv @ q,
        )[0]
        x_ddot = (
            mass_matrix_inv_q
            + self.derived_mass_properties.mass_matrix_inv @ a_matrix.T @ lambda_vec
        )

        return x_ddot


def main() -> None:
    """

    """
    mrb = MultiRigidbody()
    mrb.add_body(
        1,
        np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        v_body_wrt_0_in_0=np.array([[0], [-1], [0]]),
    )
    mrb.add_body(
        1,
        np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        r_body_wrt_0_in_0=np.array([[1], [0], [0]]),
        v_body_wrt_0_in_0=np.array([[0], [1], [0]]),
    )
    mrb.add_revolute_joint(
        0,
        1,
        np.array([[0.5], [0], [0]]),
        -np.array([[0.5], [0], [0]]),
    )
    mrb.initialize_derived_mass_properties()

    def integration_wrapper(t: float, x: np.ndarray) -> np.ndarray:
        state = x[:12].reshape((12, 1))
        state_dot = x[12:].reshape((12, 1))
        mrb.state = state
        mrb.state_dot = state_dot

        state_ddot = mrb.uk_dynamics()

        return np.concatenate([state_dot, state_ddot], axis=0).flatten()

    NUM_STEPS = 1000
    dt = 0.01
    states = np.zeros((NUM_STEPS + 1, 12))
    states[0, :] = mrb.state.flatten()
    state_dots = np.zeros((NUM_STEPS + 1, 12))
    state_dots[0, :] = mrb.state_dot.flatten()
    t = np.arange(NUM_STEPS + 1) * dt
    for i in range(NUM_STEPS):
        t_span = t[i : i + 2]
        out = solve_ivp(
            integration_wrapper,
            t_span,
            np.concatenate([states[i, :], state_dots[i, :]]),
        )
        new_state = out.y[:, -1]
        states[i + 1, :] = new_state[:12]
        sigma_0_to_1 = states[i + 1, 3:6]
        if (sigma_0_to_1_norm_squared := sigma_0_to_1 @ sigma_0_to_1) > 1:
            sigma_0_to_1 = -sigma_0_to_1 / sigma_0_to_1_norm_squared
        states[i + 1, 3:6] = sigma_0_to_1
        sigma_0_to_2 = states[i + 1, 9:12]
        if (sigma_0_to_2_norm_squared := sigma_0_to_2 @ sigma_0_to_2) > 1:
            sigma_0_to_2 = -sigma_0_to_2 / sigma_0_to_2_norm_squared
        states[i + 1, 9:12] = sigma_0_to_2
        state_dots[i + 1, :] = new_state[12:]

    plt.figure()
    plt.plot(states[:, 0], states[:, 1])
    plt.plot(states[:, 6], states[:, 7])

    plt.figure()
    plt.plot(t, states[:, 3:6])
    plt.plot(t, states[:, 9:12], linestyle="--")
    plt.show()


if __name__ == "__main__":
    main()
