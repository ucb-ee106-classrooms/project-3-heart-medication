import matplotlib.pyplot as plt
import numpy as np
plt.rcParams['font.family'] = ['Arial']
plt.rcParams['font.size'] = 14
import time as time 


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
    def compute_error_metrics(self):
        if len(self.x) == 0 or len(self.x) != len(self.x_hat):
            return None, None 
        errors = []
        for true_state, est_state in zip(self.x, self.x_hat):
            true_xy = np.array(true_state[2:4])
            est_xy = np.array(est_state[2:4])
            err = np.linalg.norm(true_xy - est_xy)
            errors.append(err)
        errors = np.array(errors)
        rmse = np.sqrt(np.mean(errors**2))
        mae = np.mean(np.abs(errors))
        return rmse, mae 

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
        self.total_runtime = 0.0
        self.num_updates = 0

    def update(self, _):
        start_time = time.time()
        if len(self.x_hat) > 0:
            # TODO: Your implementation goes here!
            # You may ONLY use self.u and self.x[0] for estimation
            # raise NotImplementedError
            last_state = self.x_hat[-1]
            u = self.u[_]
            x = last_state[0]
            z = last_state[1]
            phi = last_state[2]
            vx = last_state[3]
            vz = last_state[4]
            omega = last_state[5]

            u1 = u[0]
            u2 = u[1]

            new_x = x + vx*self.dt 
            new_z = z + vz*self.dt 

            new_phi = phi+omega *self.dt 
            new_vx = vx + (-u1*np.sin(phi)/self.m)*self.dt 
            new_vz = vz + (-self.gr + u1 * np.cos(phi)/self.m)*self.dt

            new_omega = omega + (u2/self.J)*self.dt

            new_state = np.array([new_x, new_z, new_phi, new_vx, new_vz, new_omega])
            self.x_hat.append(new_state)

            
            # xi_hat = np.copy(self.x[0])
            # for i in range(len(self.x)):
            #     xi_hat = self.update_dynamics(xi_hat, self.u[i])
            
            # self.x_hat.append(xi_hat)

        end_time = time.time()
        runtime = end_time - start_time
        self.total_runtime+=runtime
        avg_runtime = self.total_runtime/(len(self.x_hat)-1)
        print(f"DEAD RECKONING DRONE AVG COMPUTE TIME: {avg_runtime} seconds")

        rmse, mae = self.compute_error_metrics()
        if rmse is not None: 
            print(f"Dead Reckoning RMSE (Drone): {rmse}")
            print(f"Dead Reckoning MAE (Drone): {mae}")
            

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
        # TODO: Your implementation goes here!
        # You may define the Q, R, and P matrices below.
        self.A = None
        self.B = None
        self.C = None
        self.Q = np.eye(6)*0.1
        self.R = np.eye(2)*3 
        self.P = np.eye(6)*2
        self.total_runtime = 0

    # noinspection DuplicatedCode
    def update(self, i):
        # if i<len(self.y) and len(self.x_hat) > 0: #and self.x_hat[-1][0] < self.x[-1][0]:
        if i<len(self.y):
            start_time = time.time()
            # TODO: Your implementation goes here!
            # You may use self.u, self.y, and self.x[0] for estimation
            # raise NotImplementedError
            last_state = self.x_hat[-1]
            u = self.u[i-1]
            x_pred = self.g(last_state,u)
            A = self.approx_A(last_state, u)
            P_pred = A @ self.P @ A.T + self.Q
            C = self.approx_C(x_pred)
            K = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + self.R)
            x_updated = x_pred + K@ (self.y[i] - self.h(x_pred))

            I = np.eye(len(self.P))
            self.P = (I-K@C) @ P_pred 
            self.x_hat.append(x_updated)

            end_time = time.time()
            runtime = end_time - start_time
            self.total_runtime+=runtime
            avg_runtime = self.total_runtime/(len(self.x_hat)-1)
            print(f"EKF DRONE AVG COMPUTE TIME: {avg_runtime} seconds")

            

            rmse, mae = self.compute_error_metrics()
            if rmse is not None: 
                print(f"EKF RMSE (Drone): {rmse}")
                print(f"EKF MAE (Drone): {mae}")

    def g(self, x, u):
        # raise NotImplementedError
        A = np.array([x[3], x[4], x[5], 0, -self.gr, 0])
        phi = x[2]
        B = np.array([[0,0], [0,0], [0,0], [-np.sin(phi)/self.m, 0], [np.cos(phi)/self.m, 0], [0,1/self.J]])
        f_x = A + B@u
        g = x+ f_x*self.dt
        return g


    def h(self, x):
        # raise NotImplementedError
        drone_x = x[0]
        drone_z = x[1]
        drone_phi = x[2]

        landmark_x = self.landmark[0]
        landmark_y = self.landmark[1]
        landmark_z = self.landmark[2]

        dx = landmark_x - drone_x
        dz = landmark_z - drone_z

        distance = np.sqrt(dx**2 + landmark_y**2 + dz**2)
        return np.array([distance, drone_phi])

    def approx_A(self, x, u):
        phi = x[2]
        u1 = u[0]

        A = np.array([
            [1,0,0,self.dt,0,0],
            [0,1,0,0,self.dt,0],
            [0,0,1,0,0,self.dt],
            [0,0,-u1*np.cos(phi)*self.dt/self.m,1,0,0],
            [0,0,-u1*np.sin(phi)*self.dt/self.m,0,1,0],
            [0,0,0,0,0,1]
        ])
        return A
    
    def approx_C(self, x):
        # raise NotImplementedError
        drone_x = x[0]
        drone_z = x[1]
        landmark_x = self.landmark[0]
        landmark_y = self.landmark[1]
        landmark_z = self.landmark[2]

        dx = landmark_x - drone_x
        dz = landmark_z - drone_z

        distance = np.sqrt(dx**2 + landmark_y**2 + dz**2)

        C = np.array([[-dx/distance, -dz/distance, 0, 0, 0, 0],[0,0,1,0,0,0]])
        return C
