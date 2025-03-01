#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
Adapted for Spring 2020 by Amay Saxena
"""
import numpy as np
from scipy.integrate import quad
import sys
from copy import copy
import matplotlib.pyplot as plt
from .configuration_space import Plan, BicycleConfigurationSpace

class SinusoidPlanner():
    def __init__(self, config_space):
        """
        Turtlebot planner that uses sequential sinusoids to steer to a goal pose.

        config_space should be a BicycleConfigurationSpace object.
        Parameters
        ----------
        l : float
            length of car
        """
        self.config_space = config_space
        self.l = config_space.robot_length
        self.max_phi = config_space.high_lims[3]
        self.max_u1 = config_space.input_high_lims[0]
        self.max_u2 = config_space.input_high_lims[1]

    def plan_to_pose(self, start_state, goal_state, dt = 0.01, delta_t=2):
        """
        Plans to a specific pose in (x,y,theta,phi) coordinates.  You 
        may or may not have to convert the state to a v state with state2v()
        You may want to plan each component separately
        so that you can reset phi in case there's drift in phi.

        You will need to edit some or all of this function to take care of
        configuration

        Parameters
        ----------
        start_state: numpy.ndarray of shape (4,) [x, y, theta, phi]
        goal_state: numpy.ndarray of shape (4,) [x, y, theta, phi]
        dt : float
            how many seconds between trajectory timesteps
        delta_t : float
            how many seconds each trajectory segment should run for

        Returns
        -------
        :obj: Plan
            See configuration_space.Plan.
        """

        print("======= Planning with SinusoidPlanner =======")

        self.plan = None
        # This bit hasn't been exhaustively tested, so you might hit a singularity anyways
        x_s, y_s, theta_s, phi_s = start_state
        x_g, y_g, theta_g, phi_g = goal_state
        max_abs_angle = max(abs(theta_g), abs(theta_s))
        min_abs_angle = min(abs(theta_g), abs(theta_s))
        if (max_abs_angle > np.pi/2) and (min_abs_angle < np.pi/2):
            raise ValueError("You'll cause a singularity here. You should add something to this function to fix it")

        if abs(phi_s) > self.max_phi or abs(phi_g) > self.max_phi:
            raise ValueError("Either your start state or goal state exceeds steering angle bounds")

        # We can only change phi up to some threshold
        self.phi_dist = min(
            abs(phi_g - self.max_phi),
            abs(phi_g + self.max_phi)
        )

        x_path =        self.steer_x(
                            start_state, 
                            goal_state, 
                            dt=dt, 
                            delta_t=delta_t
                        )

        phi_path =      self.steer_phi(
                            x_path.end_position(), 
                            goal_state,  
                            dt=dt, 
                            delta_t=delta_t
                        )
        alpha_path =    self.steer_alpha(
                            phi_path.end_position(), 
                            goal_state, 
                            dt=dt, 
                            delta_t=delta_t
                        )
        y_path =        self.steer_y(
                            alpha_path.end_position(), 
                            goal_state,
                            dt=dt,
                            delta_t=delta_t
                        )     

        self.plan = Plan.chain_paths(x_path, phi_path, alpha_path, y_path)
        return self.plan

    def plot_execution(self):
        """
        Creates a plot of the planned path in the environment. Assumes that the 
        environment of the robot is in the x-y plane, and that the first two
        components in the state space are x and y position. Also assumes 
        plan_to_pose has been called on this instance already, so that self.graph
        is populated. If planning was successful, then self.plan will be populated 
        and it will be plotted as well.
        """
        ax = plt.subplot(1, 1, 1)

        if self.plan:
            plan_x = self.plan.positions[:, 0]
            plan_y = self.plan.positions[:, 1]
            ax.plot(plan_x, plan_y, color='green')

        plt.show()

    def steer_x(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a Plan to move the turtlebot in the x direction

        Parameters
        ----------
        start_state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            current state of the turtlebot
        start_state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj: Plan
            See configuration_space.Plan.
        """
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_x = goal_state_v[0] - start_state_v[0]

        v1 = delta_x/delta_t
        v2 = 0

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def steer_phi(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the phi direction

        Parameters
        ----------
        start_state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            current state of the turtlebot
        goal_state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj: Plan
            See configuration_space.Plan.
        """
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_phi = goal_state_v[1] - start_state_v[1]

        v1=0
        v2 = delta_phi/delta_t

        path, t = [], t0

        while t<t0 + delta_t:
            path.append([t,v1,v2])
            t+=dt
        return self.v_path_to_u_path(path, start_state, dt)



    def steer_alpha(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the alpha direction.  
        Remember dot{alpha} = f(phi(t))*u_1(t) = f(frac{a_2}{omega}*sin(omega*t))*a_1*sin(omega*t)
        also, f(phi) = frac{1}{l}tan(phi)
        See the doc for more math details

        Parameters
        ----------
        start_state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            current state of the turtlebot
        goal_state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj: Plan
            See configuration_space.Plan.
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_alpha = goal_state_v[2] - start_state_v[2]

        omega = 2*np.pi / delta_t

        a2 = min(1, self.phi_dist*omega)
        f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model
        phi_fn = lambda t: (a2/omega)*np.sin(omega*t) + start_state_v[1]
        integrand = lambda t: f(phi_fn(t))*np.sin(omega*t) # The integrand to find beta
        beta1 = (omega/np.pi) * quad(integrand, 0, delta_t)[0]

        a1 = (delta_alpha*omega)/(np.pi*beta1)
              
        v1 = lambda t: a1*np.sin(omega*(t))
        v2 = lambda t: a2*np.cos(omega*(t))

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)


    def steer_y(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the y direction. 
        Remember, dot{y} = g(alpha(t))*v1 = frac{alpha(t)}{sqrt{1-alpha(t)^2}}*a_1*sin(omega*t)
        See the doc for more math details

        Parameters
        ----------
        start_state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            current state of the turtlebot
        goal_state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj: Plan
            See configuration_space.Plan.
        """
        #using alg from the Ed supplement steer_y, 2nd page 

        def f(phi):
            return 1/(self.l)*np.tan(phi)
        def g(alpha):
            return alpha/np.sqrt(1-alpha**2)
        def phi_fn(t):
            phi_0 = start_state_v[1]
            return phi_0 + (a2/(2*omega))*np.sin(2*omega*t)
        def integrand_alpha(t,a1):
            return f(phi_fn(t))*a1*np.sin(omega*t)
        def alpha_fn(t, a1):
            alpha_0 = start_state_v[2]
            return alpha_0 + quad(integrand_alpha, 0, t, (a1))[0]
        def integrand(t, a1):
            return g(alpha_fn(t, a1))*np.sin(omega*t)
        def calculate_beta1(a1):
            return (omega/np.pi)*quad(integrand, 0, delta_t, (a1))[0]
        def close_enough(guess, goal, threshold = 1e-5):
            return np.abs(guess-goal)<threshold 
        
        ########################################################################################################################################
        #1. given initial state yi, goal state yd, delta_t 

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_y = goal_state_v[3]-start_state_v[3]
        omega = 2*np.pi/delta_t

        low, high = -5, 5
        a1, a2 = (low+high)/2, self.max_u2
        G = float('inf')
        count = 0
        while low<high:
            a1 = (low+high)/2
            beta1 = calculate_beta1(a1)
            G = a1*(np.pi/omega)*beta1
            if close_enough(G, delta_y, 1e-8):
                print('found close enough')
                break
            if count>1000:
                print('too many loops')
                break 

            if G< delta_y:
                low =a1
            else:
                high = a1
            count+=1
        
        v1 = lambda t: a1*np.sin(omega*(t))
        v2 = lambda t: a2*np.cos(2*omega*(t))

        
        # breakpoint()
        #2. compute omega = 2pi/delta_t 
        
        
        # #3. set arbitary a2 (the initial guess)
        # a2 = 1
        
        # #4. beta integrated up to delta_t
        # #the beta determines how much y displacement results from a given a1, since beta1 depends on the a1, we need to iterate via binary search 
        # # to find the right a1 so that the turtlebot reaches desired yd

        # # a1_low, a1_high = 0, self.max_u1
        # # a1_low, a1_high = 0, self.max_u2
        # a1_low, a1_high = -5, 5

        # #5. binary search to find a1
        # #mb we can experiment with setting a tolerance threshold
        # tolerance = 0.05
        # # for _ in range(20):
        # a1 = (a1_low + a1_high)/2
        # alpha_fn = lambda t: np.sin(start_state_v[2] + a2/(2*omega) * np.sin(2*omega*t))
        # integrand = lambda t: alpha_fn(t) / np.sqrt(1-alpha_fn(t)**2) * a1 * np.sin(omega*t)
        # beta1 = (omega/np.pi) * quad(integrand, 0, delta_t)[0]

        # guess_y = (a1*np.pi/omega)*beta1
        # # a1, guess_y = 1, 1
        # while abs(guess_y - delta_y)>tolerance:
        # # for _ in range(20):
        #     a1 = (a1_low + a1_high)/2

        #     #compute beta 
        #     alpha_fn = lambda t: np.sin(start_state_v[2] + a2/omega * np.sin(2*omega*t))
        #     integrand = lambda t: alpha_fn(t) / np.sqrt(1-alpha_fn(t)**2) * a1 * np.sin(omega*t)
        #     beta1 = (omega/np.pi) * quad(integrand, 0, delta_t)[0]

        #     guess_y = (a1*np.pi/omega)*beta1
        #     if abs(guess_y - delta_y)<tolerance:
        #         break
        #     elif guess_y < delta_y:
        #         a1_low = a1
        #     else:
        #         a1_high = a1


        # #attempt 2 
        # # start, end = 0.3, 2
        # # a1 = (start+end)/2

        # # f = lambda phi: (1/self.l)*np.tan(phi)
        # # phi_fn = lambda t: (a2/omega)*np.sin(omega*t)+start_state_v[1]
        # # g = lambda x: x/np.sqrt(1-x**2)
        # # phi0 = start_state_v[1]
        # # alpha0 = start_state_v[2]
        # # G = lambda a1, omega, b1: a1*np.pi/omega * b1

        # # outer_result = quad(lambda tau: g(quad(lambda s: f((a2/2*omega))*np.sin(2*omega*s) + phi0)*a1*np.sin(omega*s))


        # #6. use amplitude of sine wave of a1, a2; compute v1 and v2
        # v1 = lambda t: min(a1, self.max_u1*np.cos(start_state[2]))*np.sin(omega*t)


        # v2 = lambda t: a2*np.cos(2*omega*t)
        #gen trajectory 



        path, t = [], t0
        while t< t0 + delta_t:
            path.append([t,v1(t-t0), v2(t-t0)])
            t+=dt
        return self.v_path_to_u_path(path, start_state, dt)

    def state2v(self, state):
        """
        Takes a state in (x,y,theta,phi) coordinates and returns a state of (x,phi,alpha,y)

        Parameters
        ----------
        state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            some state

        Returns
        -------
        4x1 :obj:`numpy.ndarray` 
            x, phi, alpha, y
        """
        x, y, theta, phi = state
        return np.array([x, phi, np.sin(theta), y])

    def v_path_to_u_path(self, path, start_state, dt):
        """
        convert a trajectory in v commands to u commands

        Parameters
        ----------
        path : :obj:`list` of (float, float, float)1740277617.357253
            list of (time, v1, v2) commands
        start_state : numpy.ndarray of shape (4,) [x, y, theta, phi]
            starting state of this trajectory
        dt : float
            how many seconds between timesteps in the trajectory

        Returns
        -------
        :obj: Plan
            See configuration_space.Plan.
        """
        def v2cmd(v1, v2, state):
            u1 = v1/np.cos(state[2])
            u2 = v2
            return [u1, u2]

        curr_state = start_state
        positions = []
        times = []
        open_loop_inputs = []
        for i, (t, v1, v2) in enumerate(path):
            cmd_u = v2cmd(v1, v2, curr_state)
            positions.append(curr_state)
            open_loop_inputs.append(cmd_u)
            times.append(t)

            x, y, theta, phi = curr_state
            linear_velocity, steering_rate = cmd_u
            curr_state = [
                x     + np.cos(theta)               * linear_velocity*dt,
                y     + np.sin(theta)               * linear_velocity*dt,
                theta + np.tan(phi) / float(self.l) * linear_velocity*dt,
                phi   + steering_rate*dt
            ]

        return Plan(np.array(times), np.array(positions), np.array(open_loop_inputs), dt=dt)

def main():

    """Use this function if you'd like to test without ROS.
    """
    start = np.array([1, 1, 0, 0]) 
    goal = np.array([2, 1.3, 0.7, 0])
    xy_low = [0, 0]
    xy_high = [5, 5]
    phi_max = 0.6
    u1_max = 2
    u2_max = 3
    obstacles = []

    config = BicycleConfigurationSpace( xy_low + [-1000, -phi_max],
                                        xy_high + [1000, phi_max],
                                        [-u1_max, -u2_max],
                                        [u1_max, u2_max],
                                        obstacles,
                                        0.15)

    planner = SinusoidPlanner(config)
    plan = planner.plan_to_pose(start, goal, 0.01, 2.0)
    planner.plot_execution()

if __name__ == '__main__':
    main()
