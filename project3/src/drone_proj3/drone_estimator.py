import matplotlib.pyplot as plt
import numpy as np
plt.rcParams['font.family'] = ['Arial']
plt.rcParams['font.size'] = 14


class Estimator:
    """A base class to represent an estimator.

    This module contains the basic elements of an estimator, on which the
    subsequent DeadReckoning, Kalman Filter, and Extended Kalman Filter classes
    will be based on. A plotting function is provided to visualize the
    estimation results in real time.

    Attributes:
    ----------
        u : list
            A list of system inputs, where, for the ith data point u[i],
            u[i][1] is the thrust of the quadrotor
            u[i][2] is right wheel rotational speed (rad/s).
        x : list
            A list of system states, where, for the ith data point x[i],
            x[i][0] is translational position in x (m),
            x[i][1] is translational position in z (m),
            x[i][2] is the bearing (rad) of the quadrotor
            x[i][3] is translational velocity in x (m/s),
            x[i][4] is translational velocity in z (m/s),
            x[i][5] is angular velocity (rad/s),
        y : list
            A list of system outputs, where, for the ith data point y[i],
            y[i][1] is distance to the landmark (m)
            y[i][2] is relative bearing (rad) w.r.t. the landmark
        x_hat : list
            A list of estimated system states. It should follow the same format
            as x.
        dt : float
            Update frequency of the estimator.
        fig : Figure
            matplotlib Figure for real-time plotting.
        axd : dict
            A dictionary of matplotlib Axis for real-time plotting.
        ln* : Line
            matplotlib Line object for ground truth states.
        ln_*_hat : Line
            matplotlib Line object for estimated states.
        canvas_title : str
            Title of the real-time plot, which is chosen to be estimator type.

    Notes
    ----------
        The landmark is positioned at (0, 5, 5).
    """
    # noinspection PyTypeChecker
    def __init__(self, is_noisy=False):
        self.u = []
        self.x = []
        self.y = []
        self.x_hat = []  # Your estimates go here!
        self.t = []
        self.fig, self.axd = plt.subplot_mosaic(
            [['xz', 'phi'],
             ['xz', 'x'],
             ['xz', 'z']], figsize=(20.0, 10.0))
        self.ln_xz, = self.axd['xz'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_xz_hat, = self.axd['xz'].plot([], 'o-c', label='Estimated')
        self.ln_phi, = self.axd['phi'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_phi_hat, = self.axd['phi'].plot([], 'o-c', label='Estimated')
        self.ln_x, = self.axd['x'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_x_hat, = self.axd['x'].plot([], 'o-c', label='Estimated')
        self.ln_z, = self.axd['z'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_z_hat, = self.axd['z'].plot([], 'o-c', label='Estimated')
        self.canvas_title = 'N/A'

        # Defined in dynamics.py for the dynamics model
        # m is the mass and J is the moment of inertia of the quadrotor 
        self.gr = 9.81 
        self.m = 0.92
        self.J = 0.0023
        # These are the X, Y, Z coordinates of the landmark
        self.landmark = (0, 5, 5)

        # This is a (N,12) where it's time, x, u, then y_obs 
        if is_noisy:
            with open('noisy_data.npy', 'rb') as f:
                self.data = np.load(f)
        else:
            with open('data.npy', 'rb') as f:
                self.data = np.load(f)

        self.dt = self.data[-1][0]/self.data.shape[0]


    def run(self):
        for i, data in enumerate(self.data):
            self.t.append(np.array(data[0]))
            self.x.append(np.array(data[1:7]))
            self.u.append(np.array(data[7:9]))
            self.y.append(np.array(data[9:12]))
            if i == 0:
                self.x_hat.append(self.x[-1])
            else:
                self.update(i)
        return self.x_hat

    def update(self, _):
        raise NotImplementedError

    def plot_init(self):
        self.axd['xz'].set_title(self.canvas_title)
        self.axd['xz'].set_xlabel('x (m)')
        self.axd['xz'].set_ylabel('z (m)')
        self.axd['xz'].set_aspect('equal', adjustable='box')
        self.axd['xz'].legend()
        self.axd['phi'].set_ylabel('phi (rad)')
        self.axd['phi'].set_xlabel('t (s)')
        self.axd['phi'].legend()
        self.axd['x'].set_ylabel('x (m)')
        self.axd['x'].set_xlabel('t (s)')
        self.axd['x'].legend()
        self.axd['z'].set_ylabel('z (m)')
        self.axd['z'].set_xlabel('t (s)')
        self.axd['z'].legend()
        plt.tight_layout()

    def plot_update(self, _):
        self.plot_xzline(self.ln_xz, self.x)
        self.plot_xzline(self.ln_xz_hat, self.x_hat)
        self.plot_philine(self.ln_phi, self.x)
        self.plot_philine(self.ln_phi_hat, self.x_hat)
        self.plot_xline(self.ln_x, self.x)
        self.plot_xline(self.ln_x_hat, self.x_hat)
        self.plot_zline(self.ln_z, self.x)
        self.plot_zline(self.ln_z_hat, self.x_hat)

    def plot_xzline(self, ln, data):
        if len(data):
            x = [d[0] for d in data]
            z = [d[1] for d in data]
            ln.set_data(x, z)
            self.resize_lim(self.axd['xz'], x, z)

    def plot_philine(self, ln, data):
        if len(data):
            t = self.t
            phi = [d[2] for d in data]
            ln.set_data(t, phi)
            self.resize_lim(self.axd['phi'], t, phi)

    def plot_xline(self, ln, data):
        if len(data):
            t = self.t
            x = [d[0] for d in data]
            ln.set_data(t, x)
            self.resize_lim(self.axd['x'], t, x)

    def plot_zline(self, ln, data):
        if len(data):
            t = self.t
            z = [d[1] for d in data]
            ln.set_data(t, z)
            self.resize_lim(self.axd['z'], t, z)

    # noinspection PyMethodMayBeStatic
    def resize_lim(self, ax, x, y):
        xlim = ax.get_xlim()
        ax.set_xlim([min(min(x) * 1.05, xlim[0]), max(max(x) * 1.05, xlim[1])])
        ylim = ax.get_ylim()
        ax.set_ylim([min(min(y) * 1.05, ylim[0]), max(max(y) * 1.05, ylim[1])])

    def update_dynamics(self, state, u):
        x_velo, z_velo, phi_velo = state[3:]
        phi = state[2]
        m, gr, J = self.m, self.gr, self.J

        dyn_mat = np.array(([0, 0],
                            [0, 0],
                            [0, 0],
                            [-np.sin(phi)/m, 0],
                            [np.cos(phi)/m, 0],
                            [0, 1/J]))

        state_dot = np.array(([x_velo, z_velo, phi_velo, 0, -gr, 0]))
        
        dyn_upd = state_dot + dyn_mat @ u

        return state + dyn_upd * self.dt


class OracleObserver(Estimator):
    """Oracle observer which has access to the true state.

    This class is intended as a bare minimum example for you to understand how
    to work with the code.

    Example
    ----------
    To run the oracle observer:
        $ python drone_estimator_node.py --estimator oracle_observer
    """
    def __init__(self, is_noisy=False):
        super().__init__(is_noisy)
        self.canvas_title = 'Oracle Observer'

    def update(self, _):
        self.x_hat.append(self.x[-1])


class DeadReckoning(Estimator):
    """Dead reckoning estimator.

    Your task is to implement the update method of this class using only the
    u attribute and x0. You will need to build a model of the unicycle model
    with the parameters provided to you in the lab doc. After building the
    model, use the provided inputs to estimate system state over time.

    The method should closely predict the state evolution if the system is
    free of noise. You may use this knowledge to verify your implementation.

    Example
    ----------
    To run dead reckoning:
        $ python drone_estimator_node.py --estimator dead_reckoning
    """
    def __init__(self, is_noisy=False):
        super().__init__(is_noisy)
        self.canvas_title = 'Dead Reckoning'

    def update(self, _):
        if len(self.x_hat) > 0:
            xi_hat = np.copy(self.x[0])
            for i in range(len(self.x)):
                xi_hat = self.update_dynamics(xi_hat, self.u[i])
            
            self.x_hat.append(xi_hat)

# noinspection PyPep8Naming
class ExtendedKalmanFilter(Estimator):
    """Extended Kalman filter estimator.

    Your task is to implement the update method of this class using the u
    attribute, y attribute, and x0. You will need to build a model of the
    unicycle model and linearize it at every operating point. After building the
    model, use the provided inputs and outputs to estimate system state over
    time via the recursive extended Kalman filter update rule.

    Hint: You may want to reuse your code from DeadReckoning class and
    KalmanFilter class.

    Attributes:
    ----------
        landmark : tuple
            A tuple of the coordinates of the landmark.
            landmark[0] is the x coordinate.
            landmark[1] is the y coordinate.
            landmark[2] is the z coordinate.

    Example
    ----------
    To run the extended Kalman filter:
        $ python drone_estimator_node.py --estimator extended_kalman_filter
    """
    def __init__(self, is_noisy=False):
        super().__init__(is_noisy)
        self.canvas_title = 'Extended Kalman Filter'
        # hardcoded landmark position
        self.lx = 0
        self.ly = 5
        self.lz = 5
        # You may define the A, C, Q, R, and P matrices below.
        n, self.n = 6, 6 # dim of x
        p, self.p = 2, 2 # dim of outputs
        m, self.m = 2, 2 # dim of inputs

        # TODO: these are closed-form partial derivative matrices, need calc
        self.A = None
        self.B = None
        self.C = None
        
        # TODO: tune these
        self.Q = np.identity(n)*1250
        self.Q[0][0] *= 0.00008 # x weighting #flattens 
        self.Q[1][1] *= 17 # z weighting
        self.Q[2][2] *= 1 # phi weighting
        self.R = np.identity(p)*0.00048980 #smoothing
        self.R[1][1] = 1.24
        # P lives in n by function of Kalman filter: P+1 = A*P*A.T + Q
        self.P_0 = np.identity(n)*1.9
        self.P_0[1][1]*=0.9
        self.P_t = [self.P_0]

        # self.A = np.eye(6)
        # self.Q = np.diag([1,1,1,0.1, 0.1, 0.1])
        # self.R = np.diag([30,10])
        # self.P_0 = np.diag([5,5,5,1,1,1])
        # self.P_t = [self.P_0]


    # NOTE: per Teja we don't need to use g_lin; taken care of by EKF
    # Meaning, we do not use B -- at least he didn't in his code
    # TODO: think bug in implementation; getting odd deviation in z and doesn't change even if set A = all 0s
    # noinspection DuplicatedCode
    def update(self, i):
        if len(self.x_hat) > 0: #and self.x_hat[-1][0] < self.x[-1][0]:
            # You may use self.u, self.y, and self.x[0] for estimation
            A, B, C, Q, R, P_0 = self.A, self.B, self.C, self.Q, self.R, self.P_0
            I = np.identity(self.n)
            
            if len(self.x_hat) == 0:
                self.x_hat.append(np.copy(self.x[0]))

            #breakpoint()
            i = (len(self.x) - 2)

            xi_hat = np.copy(self.x_hat[i])
            Pi = np.copy(self.P_t[i])
            ui = self.u[i]
            x_iP1_i = self.g(xi_hat, ui) 
            A_iP1 = self.approx_A(xi_hat, ui)
            #breakpoint()
            P_iP1_i = A_iP1 @ Pi @ A_iP1.T + Q
            C_iP1 = self.approx_C(x_iP1_i)
            K_iP1 = P_iP1_i @ C_iP1.T @ np.linalg.inv(C_iP1 @ P_iP1_i @ C_iP1.T + R)
            
            y_iP1 = self.y[i+1]
            # no need pass y to h; point of H is to linearize y around point
            # then we subtract from the measured outputs to see how accurate we are
            xiP1_hat = x_iP1_i + K_iP1 @ (y_iP1 - self.h(x_iP1_i))
            P_iP1 = (I - K_iP1 @ C_iP1) @ P_iP1_i

            self.x_hat.append(xiP1_hat)
            self.P_t.append(P_iP1)
            #breakpoint()
    
    #### UNUSED FUNCTION -- the EKF algorithm given linearizes dynamics for us ####
    # Leaving it here -- difference between linear dynamics and NL confusing
    # linear dynamics appear to need x, u, and point (x_s, u_s)
    # x could be xi_hat, u and u_s could be ui, but not sure about x_s
    # maybe from C, the first-order deriv of measurement model h along x?
    def g_lin(self, x, u, x_s, u_s):
        A_bar = self.approx_A(x_s, u_s)
        B_bar = self.approx_B(x_s, u_s)
        E = self.g(x_s, u_s) - A_bar @ x_s - B_bar @ u_s
        return A_bar @ x + B_bar @ u + E

    def g(self, x, u):
        dt, m, g, J = self.dt, self.m, self.gr, self.J
        x, z, phi, x_v, z_v, phi_v = x[0], x[1], x[2], x[3], x[4], x[5]
        u1, u2 = u[0], u[1]
        g = np.array([x + x_v*dt,
                     z + z_v*dt,
                     phi + phi_v*dt,
                     x_v - (u1*dt*np.sin(phi)/m),
                     z_v - g*dt + (u1*dt*np.cos(phi)/m),
                     phi_v + (u2*dt)/J])

        return g

    # meant to pass x_hat
    def h(self, state):
        lx, ly, lz = self.lx, self.ly, self.lz
        x, z, rel_phi = state[0], state[1], state[2]
        est_dist = np.sqrt((lx - x)**2 + ly**2 + (lz - z)**2)
        h = np.array([est_dist,
                      rel_phi])
        return h

    def approx_A(self, x, u):
        dt, m = self.dt, self.m
        phi = x[2]
        u1, u2 = u[0], u[1]
        # should be transpose?
        A = np.array([[1, 0, 0,                      dt, 0, 0],
                      [0, 1, 0,                      0, dt, 0],
                      [0, 0, 1,                      0, 0, dt],
                      [0, 0, -(u1*dt*np.cos(phi)/m), 1, 0, 0],
                      [0, 0, -(u1*dt*np.sin(phi)/m), 0, 1, 0],
                      [0, 0, 0,                      0, 0, 1]])
        return A

    def approx_B(self, x, u):
        dt, m, J = self.dt, self.m, self.J
        phi = x[2]
        B = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [-dt*np.sin(phi)/m, dt*np.cos(phi)/m],
                      [0, dt/J]])
        return B
    
    def approx_C(self, state):
        lx, ly, lz = self.lx, self.ly, self.lz
        x, z = state[0], state[1]
        euc_dist = np.sqrt((lx-x)**2 + ly**2 + (lz-z)**2)
        d1 = -(lx - x)/euc_dist
        d2 = -(lz - z)/euc_dist
        #breakpoint()
        C = np.array([[d1, d2, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0]])
        return C