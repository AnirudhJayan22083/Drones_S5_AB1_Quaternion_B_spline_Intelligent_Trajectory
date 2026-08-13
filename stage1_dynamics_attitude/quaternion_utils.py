"""
Minimal quaternion math used by the attitude controller.

Convention: quaternions are 4-vectors [w, x, y, z] (scalar-first), matching
MuJoCo's convention for ``qpos[3:7]`` and ``framequat`` sensors. All
quaternions are assumed unit-norm; ``quat_normalize`` is provided to correct
for numerical drift.
"""

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Return q scaled to unit norm."""
    n = np.linalg.norm(q)
    return q / n


def quat_conj(q: np.ndarray) -> np.ndarray:
    """Conjugate (= inverse, for unit quaternions) of q = [w, x, y, z]."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a ⊗ b, both [w, x, y, z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_error(q: np.ndarray, q_des: np.ndarray) -> np.ndarray:
    """
    Error quaternion representing the rotation from the desired attitude to
    the current attitude, expressed in the desired-body frame:

        q_e = q_des^{-1} ⊗ q

    q_e -> [1, 0, 0, 0] as q -> q_des. This is the quantity used by
    ``AttitudeController`` (Fresk & Nikolakopoulos, 2013 style law).
    """
    return quat_mul(quat_conj(q_des), q)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Build a [w, x, y, z] quaternion from roll/pitch/yaw (radians), applied
    in the ZYX (yaw, then pitch, then roll) order. Convenience helper for
    specifying human-readable attitude setpoints in demos/tests.
    """
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_euler_deg(q: np.ndarray) -> np.ndarray:
    """Convert a [w, x, y, z] quaternion to [roll, pitch, yaw] in degrees (ZYX order)."""
    w, x, y, z = quat_normalize(q)

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees([roll, pitch, yaw])


def quat_from_rotmat(R: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix (body -> world) to a unit quaternion
    [w, x, y, z]. Shepperd's method: picks the numerically stable branch
    based on the largest of {trace, R[0,0], R[1,1], R[2,2]} to avoid
    dividing by a near-zero term.

    Used by Stage 2's position controller, which builds R from a desired
    thrust direction (body +Z) and a desired heading (body +X projection)
    and needs the resulting attitude as q_des for ``AttitudeController``.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return quat_normalize(np.array([w, x, y, z]))


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix (body -> world) for q = [w, x, y, z]."""
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
