from __future__ import division, print_function
import numpy as np
import quaternion

from dmp.canonical_system import CanonicalSystem


class RotationDMP:
    def __init__(self, n_bfs=10, alpha=48.0, beta=None, cs_alpha=None, cs=None):
        self.n_bfs = n_bfs  # basis functions
        self.alpha = alpha
        self.beta = beta if beta is not None else self.alpha / 4
        self.cs = cs if cs is not None else CanonicalSystem(alpha=cs_alpha if cs_alpha is not None else self.alpha / 2)

        # Centres of the Gaussian basis functions
        self.c = np.exp(-self.cs.alpha * np.linspace(0, 1, self.n_bfs))

        # Variance of the Gaussian basis functions
        self.h = 1.0 / np.gradient(self.c) ** 2

        # Scaling factor
        self.Dp = np.identity(3)

        # Initially weights are zero (no forcing term)
        self.w = np.zeros((3, self.n_bfs))

        # Initial- and goal positions
        self.q0 = np.quaternion(0, 0, 0, 0)
        self.gq = np.quaternion(0, 0, 0, 0)
        self.reset()

    def step(self, x, dt, tau):
        def fp(xj):
            psi = np.exp(-self.h * (xj - self.c) ** 2)
            return self.Dp.dot(self.w.dot(psi) / psi.sum() * xj)

        self.etaDot = (self.alpha * (self.beta * 2 * np.quaternion.log(
            self.gq * np.quaternion.conjugate(self.q)) - self.eta) + np.quaternion(0, fp(x)[0], fp(x)[1],
                                                                                   fp(x)[2])) / tau

        # Integrate acceleration to obtain angular velocity
        self.eta += self.etaDot * dt

        # Obtain rotation from angular velocity
        self.q = (np.quaternion.exp((dt / 2) * self.eta / tau) * self.q)

        return self.q, self.eta, self.etaDot

    def rollout(self, ts, tau):
        self.reset()

        if np.isscalar(tau):
            tau = np.full_like(ts, tau)

        x = self.cs.rollout(ts, tau)  # Integrate canonical system
        dt = np.gradient(ts)  # Differential time vector

        n_steps = len(ts)
        q = []
        dq = []
        ddq = []
        for step in range(n_steps):
            q.append(np.quaternion(0, 0, 0, 0))
            dq.append(np.quaternion(0, 0, 0, 0))
            ddq.append(np.quaternion(0, 0, 0, 0))

        for i in range(n_steps):
            q[i], dq[i], ddq[i] = self.step(x[i], dt[i], tau[i])

        return q, dq, ddq

    def reset(self):
        self.q = np.quaternion(self.q0)
        self.eta = np.quaternion(0, 0, 0, 0)
        self.etaDot = np.quaternion(0, 0, 0, 0)

    def angular_vel(q0, q1, q2, dt):
        return (2 / dt) * np.array([0, q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y,
                                    q1.w * q2.y + q1.x * q2.z - q1.y * q2.x - q1.z * q2.x,
                                    q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w])

    def train(self, rotations, ts, tau):
        q = []

        for r in rotations:
            q.append(np.quaternion(r[0], r[1], r[2], r[3]))

        # Sanity-check input
        if len(q) != len(ts):
            raise RuntimeError("len(q) != len(ts)")

        # Initial- and goal positions
        self.q0 = np.quaternion(q[0])
        self.gq = np.quaternion(q[-1])

        # Differential time vector
        dt = np.gradient(ts)[:, np.newaxis]

        # Scaling factor
        self.Dp = np.diag(quaternion.as_vector_part(2 * np.quaternion.log(self.gq * np.quaternion.conjugate(self.q0))))
        Dp_inv = np.linalg.inv(self.Dp)
        # Desired velocities and accelerations
        d_q = [np.quaternion(0, 0, 0, 0)]
        dd_q = [np.quaternion(0, 0, 0, 0)]

        for i in range(len(q) - 1):
            # d_q.append(np.quaternion(self.angular_vel(q[i],q[i+1],dt[i])[0],self.angular_vel(q[i],q[i+1],dt[i])[1],self.angular_vel(q[i],q[i+1],dt[i])[2],self.angular_vel(q[i],q[i+1],dt[i])[3]))
            d_q.append(2 * np.quaternion.log(q[i + 1] * np.quaternion.conjugate(q[i])) / dt[i])
        # d_q=quaternion.calculus.fd_derivative(q,dt)
        for i in range(len(d_q) - 1):
            dd_q.append((d_q[i + 1] - d_q[i]) / dt[i])
        dd_q.append(np.quaternion(0, 0, 0, 0))

        # Integrate canonical system
        x = self.cs.rollout(ts, tau)

        # Set up system of equations to solve for weights
        def features(xj):
            psi = np.exp(-0.5 * self.h * (xj - self.c) ** 2)
            return xj * psi / psi.sum()

        def forcing(j):
            return Dp_inv.dot(tau ** 2 * np.hstack(quaternion.as_vector_part(dd_q[j])) - self.alpha *
                              (self.beta * np.hstack(quaternion.as_vector_part(
                                  2 * np.quaternion.log(self.gq * np.quaternion.conjugate(q[j])))) -
                               tau * np.hstack(quaternion.as_vector_part(d_q[j]))))

        A = np.stack(features(xj) for xj in x)
        f = np.stack(forcing(j) for j in range(len(ts)))
        # Least squares solution for Aw = f (for each column of f)

        self.w = np.linalg.lstsq(A, f, rcond=None)[0].T

        # Cache variables for later inspection
        self.train_q = q
        self.train_d_q = d_q
        self.train_dd_q = dd_q
