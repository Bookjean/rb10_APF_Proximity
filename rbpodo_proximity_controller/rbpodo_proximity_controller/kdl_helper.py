"""KDL chain builder and FK/Jacobian utilities for RB10-1300E."""

import numpy as np
import PyKDL


# Joint definitions from joint.yaml (rb10_1300e)
# Each entry: (origin_xyz, axis_xyz)
JOINT_DEFS = [
    # base: link0 -> link1
    {"origin": [0.0, 0.0, 0.0], "axis": [0, 0, 1]},
    # shoulder: link1 -> link2
    {"origin": [0.0, 0.0, 0.197], "axis": [0, 1, 0]},
    # elbow: link2 -> link3
    {"origin": [0.0, -0.1875, 0.6127], "axis": [0, 1, 0]},
    # wrist1: link3 -> link4
    {"origin": [0.0, 0.1484, 0.57015], "axis": [0, 1, 0]},
    # wrist2: link4 -> link5
    {"origin": [0.0, -0.11715, 0.0], "axis": [0, 0, 1]},
    # wrist3: link5 -> link6
    {"origin": [0.0, 0.0, 0.11715], "axis": [0, 1, 0]},
]


def build_chain(n_joints):
    """Build a KDL chain with the first n_joints of RB10-1300E.

    n_joints=6 gives the full chain link0->link6.
    n_joints=3 gives link0->link3.

    URDF convention: T_segment = Trans(origin_i) * Rot(axis_i, q_i)
    KDL Segment:     T_segment = Rot(axis_i, q_i) * f_tip_i
    So we set f_tip_i = Trans(origin_{i+1}), shifting origins by one.
    The last segment gets identity f_tip.
    (Assumes origin_0 is identity, which holds for RB10-1300E.)
    """
    chain = PyKDL.Chain()
    for i in range(n_joints):
        ax, ay, az = JOINT_DEFS[i]["axis"]
        joint = PyKDL.Joint(
            f"j{i}", PyKDL.Vector.Zero(),
            PyKDL.Vector(ax, ay, az), PyKDL.Joint.RotAxis,
        )
        if i + 1 < n_joints:
            ox, oy, oz = JOINT_DEFS[i + 1]["origin"]
            f_tip = PyKDL.Frame(PyKDL.Rotation.Identity(),
                                PyKDL.Vector(ox, oy, oz))
        else:
            f_tip = PyKDL.Frame.Identity()
        chain.addSegment(PyKDL.Segment(f"seg{i}", joint, f_tip))
    return chain


def build_ee_chain(ee_offset=None):
    """Build full 6-DOF chain (link0 -> link6), with optional EE offset.

    ee_offset: [x, y, z] in link6 local frame. Adds a fixed segment
    so FK and Jacobian automatically account for the offset.
    """
    chain = build_chain(6)
    if ee_offset is not None:
        f_tip = PyKDL.Frame(PyKDL.Rotation.Identity(),
                            PyKDL.Vector(*ee_offset))
        chain.addSegment(PyKDL.Segment("seg_ee_offset", PyKDL.Joint(), f_tip))
    return chain


def build_link3_chain():
    """Build 3-DOF chain (link0 -> link3)."""
    return build_chain(3)


def kdl_frame_to_pos_rot(frame):
    """Extract position (3,) and rotation (3,3) numpy arrays from KDL Frame."""
    pos = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
    rot = np.array([
        [frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
        [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
        [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]],
    ])
    return pos, rot


def kdl_jacobian_to_numpy(jac, n_joints):
    """Convert KDL Jacobian to numpy array (6, n_joints)."""
    J = np.zeros((6, n_joints))
    for i in range(6):
        for j in range(n_joints):
            J[i, j] = jac[i, j]
    return J


def rpy_to_rotation_matrix(roll, pitch, yaw):
    """Convert RPY angles to a 3x3 rotation matrix."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])
    return R


def compute_fk(chain, q_array):
    """Compute forward kinematics. Returns KDL Frame."""
    n = chain.getNrOfJoints()
    fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
    q_kdl = PyKDL.JntArray(n)
    for i in range(n):
        q_kdl[i] = q_array[i]
    frame = PyKDL.Frame()
    fk_solver.JntToCart(q_kdl, frame)
    return frame


def compute_jacobian(chain, q_array):
    """Compute Jacobian (6 x n_joints). Returns numpy array."""
    n = chain.getNrOfJoints()
    jac_solver = PyKDL.ChainJntToJacSolver(chain)
    q_kdl = PyKDL.JntArray(n)
    for i in range(n):
        q_kdl[i] = q_array[i]
    jac_kdl = PyKDL.Jacobian(n)
    jac_solver.JntToJac(q_kdl, jac_kdl)
    return kdl_jacobian_to_numpy(jac_kdl, n)
