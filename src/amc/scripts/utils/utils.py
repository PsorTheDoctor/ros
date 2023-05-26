import numpy as np
import math
from controller.phase_variable import PhaseVariable


def euler2quat(data):
    """
    Convert data from Euler angles to quaternions.
    Parameters
    ----------
    data : np.ndarray
    (n x 3) matrix
    """
    data_q = np.zeros((len(data), 4))
    i = 0
    for o in data:
        roll = o[0]
        pitch = o[1]
        yaw = o[2]
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        data_q[i] = [qw, qx, qy, qz]
        i += 1

    return data_q


def axis2quat(data):
    """
    Convert data from Angle axis to quaternions.
    Parameters
    ----------
    data : np.ndarray
    (n x 3) matrix
    """
    data_q = np.empty((len(data), 4))

    for i, d in enumerate(data):
        angle = np.linalg.norm(d)
        axis_normed = d / angle
        s = math.sin(angle / 2)
        data_q[i] = [math.cos(angle / 2), s * axis_normed[0], s * axis_normed[1], s * axis_normed[2]]

    return data_q


def radialBasis(alpha, n_bfs, size):
    """
    Calculate normalized radial basis function.
    Parameters
    ----------
    alpha : float

    n_bfs : float
        No. of basis functions
    """
    pv = PhaseVariable()
    # Centres of the Gaussian basis functions
    c = np.exp(alpha * np.linspace(0, 1, n_bfs))

    # Variance of the Gaussian basis functions
    h = 1.0 / np.gradient(c) ** 2

    tau = 0.002 * size
    ts = np.arange(0, tau, 0.002)
    s = pv.rollout(ts)  # Integrate canonical system

    def features(xj):
        w = np.exp(-0.5 * h * (xj - c) ** 2)
        return w / w.sum()

    g = np.stack([features(xj) for xj in s])
    return g


def learnParameters(dof, qs, qd, qv, gamma, g, t, demo_p, actual_vel, force, actual_acceleration):
    """
    Calculate the initial parameters ks, kd, v.
    Parameters
    ----------
    dof : float
        Degrees of freedom;
    qs : float
         Learning rate s;
    qd : float
        Learning rate d;
    qv : float
        Learning rate v;
    gamma : float
        Tracking error coefficient;
    demo_p : np.ndarray
    (n x 3) matrix
    """
    # Initialize
    ks = np.empty((len(demo_p), dof))
    kd = np.empty((len(demo_p), dof))
    v = np.empty((len(demo_p), dof))
    tau1 = np.empty((len(demo_p), dof))

    n_bfs = 3

    # Computing velocity
    dt = np.gradient(t)[:, np.newaxis]
    d_p = np.gradient(demo_p, axis=0) / dt
    pos = demo_p[0, :]
    velocity = np.zeros(dof)
    acceleration = np.zeros(dof)
    mass = 1
    spring = 40#50
    damper = 2 * math.sqrt(mass * (spring+270))
    for j in range(len(demo_p)):
        pos_diff = pos - demo_p[j, :]
        vel_diff = velocity - actual_vel[j, :]
        tra_diff = gamma * pos_diff + vel_diff
        ks[j, :] = qs * tra_diff * pos_diff * g[j, n_bfs - dof:n_bfs]
        kd[j, :] = qd * tra_diff * vel_diff * g[j, n_bfs - dof:n_bfs]
        v[j, :] = qv * tra_diff * g[j, n_bfs - dof:n_bfs]
        tau1[j, :] = -(ks[j, :] * pos_diff + kd[j, :] * vel_diff) -v[j,:]
        # tau1[j, :] = force[j, :]
        acceleration = (tau1[j, :] - spring * pos - damper * velocity) / mass
        velocity = velocity + acceleration * dt[j]
        pos = pos + velocity * dt[j]

    return ks, kd, v, tau1


def createCircle(demo_p):
    """
    Create a 3d circle trajectory.
    Parameters
    ----------
    demo_p : np.ndarray
    (n x 3) matrix
    """
    r = 2  # radius of the circle
    theta = np.pi / 4  # "tilt" of the circle
    phirange = np.linspace(0, 2 * np.pi - 0.1, len(demo_p))  # to make a full circle

    # center of the circle
    center = [2, 2, 1]

    # computing the values of the circle in spherical coordinates and converting them
    # back to cartesian
    for j, phi in enumerate(phirange):
        x = r * np.cos(theta) * np.cos(phi) + center[0]
        y = r * np.sin(phi) + center[1]
        z = r * np.sin(theta) * np.cos(phi) + center[2]
        # space[x,y,z]=1
        demo_p[j][0] = x
        demo_p[j][1] = y
        demo_p[j][2] = z
    return demo_p
