import rospy
import time
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams['font.family'] = ['FreeSans', 'Helvetica', 'Arial']
plt.rcParams['font.size'] = 14


class Estimator:
    """A base class to represent an estimator.

    This module contains the basic elements of an estimator, on which the
    subsequent DeadReckoning, Kalman Filter, and Extended Kalman Filter classes
    will be based on. A plotting function is provided to visualize the
    estimation results in real time.

    Attributes:
    ----------
        d : float
            Half of the track width (m) of TurtleBot3 Burger.
        r : float
            Wheel radius (m) of the TurtleBot3 Burger.
        u : list
            A list of system inputs, where, for the ith data point u[i],
            u[i][0] is timestamp (s),
            u[i][1] is left wheel rotational speed (rad/s), and
            u[i][2] is right wheel rotational speed (rad/s).
        x : list
            A list of system states, where, for the ith data point x[i],
            x[i][0] is timestamp (s),
            x[i][1] is bearing (rad),
            x[i][2] is translational position in x (m),
            x[i][3] is translational position in y (m),
            x[i][4] is left wheel rotational position (rad), and
            x[i][5] is right wheel rotational position (rad).
        y : list
            A list of system outputs, where, for the ith data point y[i],
            y[i][0] is timestamp (s),
            y[i][1] is translational position in x (m) when freeze_bearing:=true,
            y[i][1] is distance to the landmark (m) when freeze_bearing:=false,
            y[i][2] is translational position in y (m) when freeze_bearing:=true, and
            y[i][2] is relative bearing (rad) w.r.t. the landmark when
            freeze_bearing:=false.
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
        sub_u : rospy.Subscriber
            ROS subscriber for system inputs.
        sub_x : rospy.Subscriber
            ROS subscriber for system states.
        sub_y : rospy.Subscriber
            ROS subscriber for system outputs.
        tmr_update : rospy.Timer
            ROS Timer for periodically invoking the estimator's update method.

    Notes
    ----------
        The frozen bearing is pi/4 and the landmark is positioned at (0.5, 0.5).
    """
    # noinspection PyTypeChecker
    def __init__(self):
        self.d = 0.08
        self.r = 0.033
        self.u = []
        self.x = []
        self.y = []
        self.x_hat = []  # Your estimates go here!
        self.dt = 0.1
        self.fig, self.axd = plt.subplot_mosaic(
            [['xy', 'phi'],
             ['xy', 'x'],
             ['xy', 'y'],
             ['xy', 'thl'],
             ['xy', 'thr']], figsize=(20.0, 10.0))
        self.ln_xy, = self.axd['xy'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_xy_hat, = self.axd['xy'].plot([], 'o-c', label='Estimated')
        self.ln_phi, = self.axd['phi'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_phi_hat, = self.axd['phi'].plot([], 'o-c', label='Estimated')
        self.ln_x, = self.axd['x'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_x_hat, = self.axd['x'].plot([], 'o-c', label='Estimated')
        self.ln_y, = self.axd['y'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_y_hat, = self.axd['y'].plot([], 'o-c', label='Estimated')
        self.ln_thl, = self.axd['thl'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_thl_hat, = self.axd['thl'].plot([], 'o-c', label='Estimated')
        self.ln_thr, = self.axd['thr'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_thr_hat, = self.axd['thr'].plot([], 'o-c', label='Estimated')
        self.canvas_title = 'N/A'
        self.sub_u = rospy.Subscriber('u', Float32MultiArray, self.callback_u)
        self.sub_x = rospy.Subscriber('x', Float32MultiArray, self.callback_x)
        self.sub_y = rospy.Subscriber('y', Float32MultiArray, self.callback_y)
        self.tmr_update = rospy.Timer(rospy.Duration(self.dt), self.update)

    def callback_u(self, msg):
        self.u.append(msg.data)

    def callback_x(self, msg):
        self.x.append(msg.data)
        if len(self.x_hat) == 0:
            self.x_hat.append(msg.data)

    def callback_y(self, msg):
        self.y.append(msg.data)

    def update(self, _):
        raise NotImplementedError

    def plot_init(self):
        self.axd['xy'].set_title(self.canvas_title)
        self.axd['xy'].set_xlabel('x (m)')
        self.axd['xy'].set_ylabel('y (m)')
        self.axd['xy'].set_aspect('equal', adjustable='box')
        self.axd['xy'].legend()
        self.axd['phi'].set_ylabel('phi (rad)')
        self.axd['phi'].legend()
        self.axd['x'].set_ylabel('x (m)')
        self.axd['x'].legend()
        self.axd['y'].set_ylabel('y (m)')
        self.axd['y'].legend()
        self.axd['thl'].set_ylabel('theta L (rad)')
        self.axd['thl'].legend()
        self.axd['thr'].set_ylabel('theta R (rad)')
        self.axd['thr'].set_xlabel('Time (s)')
        self.axd['thr'].legend()
        plt.tight_layout()

    def plot_update(self, _):
        self.plot_xyline(self.ln_xy, self.x)
        self.plot_xyline(self.ln_xy_hat, self.x_hat)
        self.plot_philine(self.ln_phi, self.x)
        self.plot_philine(self.ln_phi_hat, self.x_hat)
        self.plot_xline(self.ln_x, self.x)
        self.plot_xline(self.ln_x_hat, self.x_hat)
        self.plot_yline(self.ln_y, self.x)
        self.plot_yline(self.ln_y_hat, self.x_hat)
        self.plot_thlline(self.ln_thl, self.x)
        self.plot_thlline(self.ln_thl_hat, self.x_hat)
        self.plot_thrline(self.ln_thr, self.x)
        self.plot_thrline(self.ln_thr_hat, self.x_hat)

    def plot_xyline(self, ln, data):
        if len(data):
            x = [d[2] for d in data]
            y = [d[3] for d in data]
            ln.set_data(x, y)
            self.resize_lim(self.axd['xy'], x, y)

    def plot_philine(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            phi = [d[1] for d in data]
            ln.set_data(t, phi)
            self.resize_lim(self.axd['phi'], t, phi)

    def plot_xline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            x = [d[2] for d in data]
            ln.set_data(t, x)
            self.resize_lim(self.axd['x'], t, x)

    def plot_yline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            y = [d[3] for d in data]
            ln.set_data(t, y)
            self.resize_lim(self.axd['y'], t, y)

    def plot_thlline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            thl = [d[4] for d in data]
            ln.set_data(t, thl)
            self.resize_lim(self.axd['thl'], t, thl)

    def plot_thrline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            thr = [d[5] for d in data]
            ln.set_data(t, thr)
            self.resize_lim(self.axd['thr'], t, thr)

    # noinspection PyMethodMayBeStatic
    def resize_lim(self, ax, x, y):
        xlim = ax.get_xlim()
        ax.set_xlim([min(min(x) * 1.05, xlim[0]), max(max(x) * 1.05, xlim[1])])
        ylim = ax.get_ylim()
        ax.set_ylim([min(min(y) * 1.05, ylim[0]), max(max(y) * 1.05, ylim[1])])

    # calculate dynamics with x, u
    def calc_unicycle_dynamics(self, state, u):
        x, y, phi = state[2], state[3], state[1]
        r = self.r
        d = self.d
        u_partial = u[1:] # u contains timestamp, want remove that part

        # -r/2d works which is nice
        dyn_mat = np.array(([-r/(2*d), r/(2*d)],
                            [(r/2)*np.cos(phi), (r/2)*np.cos(phi)],
                            [(r/2)*np.sin(phi), (r/2)*np.sin(phi)],
                            [1, 0],
                            [0, 1]))
        dyn_in_partial = dyn_mat @ u_partial * self.dt
        #breakpoint()
        dyn_inputs = np.hstack(([self.dt], dyn_in_partial)) # add a dt to the front so matches form of state
        return state + dyn_inputs
    
    def compute_error_metrics(self):
        # if len(self.x) == 0 or len(self.x) != len(self.x_hat):
        #     return None, None 
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
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=oracle_observer \
            noise_injection:=true \
            freeze_bearing:=false
    """
    def __init__(self):
        super().__init__()
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
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=dead_reckoning \
            noise_injection:=true \
            freeze_bearing:=false
    For debugging, you can simulate a noise-free unicycle model by setting
    noise_injection:=false.
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Dead Reckoning'
        self.total_runtime = 0.0
        self.num_updates = 0

    def update(self, _): # while within if or while invoked by update func?
        start_time = time.time()
        

        if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0]: # what abt first iter, len(x_hat) sh = 0?
            x0 = self.x[0]
            xi_hat = np.copy(x0) # make copy
            for i in range(len(self.x)):
                xi_hat = self.calc_unicycle_dynamics(xi_hat, self.u[i])
            self.x_hat.append(xi_hat)
            #breakpoint()
            # rmse = self.calculate_rmse()
            # mae = self.calculate_mae()
            rmse, mae = self.compute_error_metrics()
            print(f"Dead Reckoning (Turtlebot) RMSE: {rmse}")
            print(f"Dead Reckoning (Turtlebot) MAE: {mae}")
        
            end_time = time.time()
            runtime = end_time - start_time
            self.total_runtime+=runtime
            self.num_updates+=1
            avg_runtime = self.total_runtime/(self.num_updates)
            print(f"DEAD RECKONING TURTLEBOT AVG COMPUTE TIME: {avg_runtime} seconds")

    def calculate_rmse(self):
        errors = np.array(self.x_hat) - np.array(self.x)
        mse = np.mean(errors**2, axis=0)
        rmse = np.sqrt(mse)
        return rmse 

    def calculate_mae(self):
        errors = np.abs(np.array(self.x_hat)-np.array(self.x))
        mae = np.mean(errors, axis=0)
        return mae  
            


class KalmanFilter(Estimator):
    """Kalman filter estimator.

    Your task is to implement the update method of this class using the u
    attribute, y attribute, and x0. You will need to build a model of the
    linear unicycle model at the default bearing of pi/4. After building the
    model, use the provided inputs and outputs to estimate system state over
    time via the recursive Kalman filter update rule.

    Attributes:
    ----------
        phid : float
            Default bearing of the turtlebot fixed at pi / 4.

    Example
    ----------
    To run the Kalman filter:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=kalman_filter \
            noise_injection:=true \
            freeze_bearing:=true
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Kalman Filter'
        self.time_step = 0
        self.phid = np.pi / 4
        self.old_x  = None
        self.total_runtime = 0
        # You may define the A, C, Q, R, and P matrices below.
        # NOTE: we no longer track phi in x, and matrices below ignore timestamp in x
        n, self.n = 4, 4 # dim of x
        p, self.p = 2, 2 # dim of outputs
        m, self.m = 2, 2 # dim of inputs
        phi, self.phi = np.pi / 4, np.pi / 4 # this is ugly and i hate it

        self.A = np.identity(n)
        # self.A[2][2] = 0
        # self.A[3][3] = 0
        r = self.r
        self.B = np.array(([(r/2)*np.cos(phi), (r/2)*np.cos(phi)],
                           [(r/2)*np.sin(phi), (r/2)*np.sin(phi)],
                           [1, 0],
                           [0, 1]))*self.dt
        self.C = np.array(([1, 0, 0, 0],
                           [0, 1, 0, 0]))
        
        # TODO: tune these
        self.Q = np.identity(n)*1e3
        self.Q[3][3] = 1e5
        self.R = np.identity(p)
        # P lives in n by function of Kalman filter: P+1 = A*P*A.T + Q
        self.old_P = np.identity(n)*1e4

    def kalman_state(self, state):
        return state[2:]

    def kalman_inputs(self, u):
        return u[1:]

    def kalman_outputs(self, y):
        # kalman filter uses freeze_bearing for linearization
        return y[1:]
    
    def unkalman_state_estim(self, x_hat, timestamp):
        return np.hstack((timestamp, self.phi, x_hat))

    # noinspection DuplicatedCode
    # noinspection PyPep8Naming
    def update(self, _):
        if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0] and len(self.u)>self.time_step:
            start_time = time.time()
            # You may use self.u, self.y, and self.x[0] for estimation
            # A, B, C, Q, R, P_0 = self.A, self.B, self.C, self.Q, self.R, self.P_0
            # I = np.identity(self.n)

            # # deep copy x[0] then toss unnecessary components
            # xi_hat = self.kalman_state(np.copy(self.x[0]))
            # Pt = np.copy(P_0)
            # timestamp = self.x[0][0]
            # for i in range(len(self.x) - 1):
            #     ui = self.kalman_inputs(self.u[i])
            #     x_iP1_i = A @ xi_hat + B @ ui
            #     P_tP1_t = A @ Pt @ A.T + Q
            #     K_tP1 = P_tP1_t @ C.T @ np.linalg.inv(C @ P_tP1_t @ C.T + R)
            #     y_tP1 = self.kalman_outputs(self.y[i + 1]) 
                
            #     # updating xi_hat, Pt here
            #     xi_hat = x_iP1_i + K_tP1 @ (y_tP1 - C @ x_iP1_i)
            #     Pt = (I - K_tP1 @ C) @ P_tP1_t
            #     timestamp += self.dt
            
            # # NOTE: need add back in bearing and timestamp at end
            # xi_hat = self.unkalman_state_estim(xi_hat, timestamp)
            # self.x_hat.append(xi_hat)
            if self.time_step == 0:
                self.old_x = self.x[0][2:]
            u = self.u[self.time_step][1:]
            new_x = self.A @ self.old_x + self.B @ u 
            self.old_P = self.A @ self.old_P @ self.A.T + self.Q 

            K = self.old_P @ self.C.T @np.linalg.inv(self.C @ self.old_P @ self.C.T + self.R)
            y = self.y[self.time_step][1:]
            new_x = new_x + K @ (y-self.C @ new_x)
            self.old_P = (np.eye(4) - K @ self.C) @ self.old_P
            term0 = self.u[self.time_step][0]
            new_x = np.array([term0, self.phid, new_x[0], new_x[1], new_x[2], new_x[3]])
            print("Updated state", new_x)

            self.x_hat.append(new_x)
            self.old_x = new_x[2:]
            self.time_step +=1 

            end_time = time.time()
            runtime = end_time - start_time
            self.total_runtime+=runtime
            avg_runtime = self.total_runtime/(len(self.x_hat)-1)



            rmse, mae = self.compute_error_metrics()
            if rmse is not None:
                print(f"KF RMSE:{rmse}")
                print(f"KF MAE: {mae}")
            # print(f"RMSE: {self.calculate_rmse()}")
            # print(f"MAE: {self.calculate_mae()}")
            print(f"KF TURTLEBOT AVG COMPUTE TIME: {avg_runtime} seconds")
    
    def calculate_rmse(self):
        errors = np.array(self.x_hat) - np.array(self.x)
        mse = np.mean(errors**2, axis=0)
        rmse = np.sqrt(mse)
        return rmse 

    def calculate_mae(self):
        errors = np.abs(np.array(self.x_hat)-np.array(self.x))
        mae = np.mean(errors, axis=0)
        return mae 

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

    Example
    ----------
    To run the extended Kalman filter:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=extended_kalman_filter \
            noise_injection:=true \
            freeze_bearing:=false
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Extended Kalman Filter'
        self.landmark = (0.5, 0.5)
        # TODO: Your implementation goes here!
        # You may define the Q, R, and P matrices below.

    # noinspection DuplicatedCode
    def update(self, _):
        if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0]:
            # TODO: Your implementation goes here!
            # You may use self.u, self.y, and self.x[0] for estimation
            raise NotImplementedError

