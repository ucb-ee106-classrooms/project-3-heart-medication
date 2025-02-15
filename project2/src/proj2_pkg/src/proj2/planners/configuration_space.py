#!/usr/bin/env python

"""
Starter code for EECS C106B Spring 2020 Project 2.
Author: Amay Saxena
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from contextlib import contextmanager

class Plan(object):
    """Data structure to represent a motion plan. Stores plans in the form of
    three arrays of the same length: times, positions, and open_loop_inputs.

    The following invariants are assumed:
        - at time times[i] the plan prescribes that we be in position
          positions[i] and perform input open_loop_inputs[i].
        - times starts at zero. Each plan is meant to represent the motion
          from one point to another over a time interval starting at 
          time zero. If you wish to append together multiple paths
          c1 -> c2 -> c3 -> ... -> cn, you should use the chain_paths
          method.
    """

    def __init__(self, times, target_positions, open_loop_inputs, dt=0.01):
        self.dt = dt
        self.times = times
        self.positions = target_positions
        self.open_loop_inputs = open_loop_inputs

    def __iter__(self):
        # I have to do this in an ugly way because python2 sucks and
        # I hate it.
        for t, p, c in zip(self.times, self.positions, self.open_loop_inputs):
            yield t, p, c

    def __len__(self):
        return len(self.times)

    def get(self, t):
        """Returns the desired position and open loop input at time t.
        """
        index = int(np.sum(self.times <= t))
        index = index - 1 if index else 0
        return self.positions[index], self.open_loop_inputs[index]

    def end_position(self):
        return self.positions[-1]

    def start_position(self):
        return self.positions[0]

    def get_prefix(self, until_time):
        """Returns a new plan that is a prefix of this plan up until the
        time until_time.
        """
        times = self.times[self.times <= until_time]
        positions = self.positions[self.times <= until_time]
        open_loop_inputs = self.open_loop_inputs[self.times <= until_time]
        return Plan(times, positions, open_loop_inputs)

    @classmethod
    def chain_paths(self, *paths):
        """Chain together any number of plans into a single plan.
        """
        def chain_two_paths(path1, path2):
            """Chains together two plans to create a single plan. Requires
            that path1 ends at the same configuration that path2 begins at.
            Also requires that both paths have the same discretization time
            step dt.
            """
            if not path1 and not path2:
                return None
            elif not path1:
                return path2
            elif not path2:
                return path1
            assert path1.dt == path2.dt, "Cannot append paths with different time deltas."
            assert np.allclose(path1.end_position(), path2.start_position()), "Cannot append paths with inconsistent start and end positions."
            times = np.concatenate((path1.times, path1.times[-1] + path2.times[1:]), axis=0)
            positions = np.concatenate((path1.positions, path2.positions[1:]), axis=0)
            open_loop_inputs = np.concatenate((path1.open_loop_inputs, path2.open_loop_inputs[1:]), axis=0)
            dt = path1.dt
            return Plan(times, positions, open_loop_inputs, dt=dt)
        chained_path = None
        for path in paths:
            chained_path = chain_two_paths(chained_path, path)
        return chained_path

@contextmanager
def expanded_obstacles(obstacle_list, delta):
    """Context manager that edits obstacle list to increase the radius of
    all obstacles by delta.
    
    Assumes obstacles are circles in the x-y plane and are given as lists
    of [x, y, r] specifying the center and radius of the obstacle. So
    obstacle_list is a list of [x, y, r] lists.

    Note we want the obstacles to be lists instead of tuples since tuples
    are immutable and we would be unable to change the radii.

    Usage:
        with expanded_obstacles(obstacle_list, 0.1):
            # do things with expanded obstacle_list. While inside this with 
            # block, the radius of each element of obstacle_list has been
            # expanded by 0.1 meters.
        # once we're out of the with block, obstacle_list will be
        # back to normal
    """
    for obs in obstacle_list:
        obs[2] += delta
    yield obstacle_list
    for obs in obstacle_list:
        obs[2] -= delta

class ConfigurationSpace(object):
    """ An abstract class for a Configuration Space. 
    
        DO NOT FILL IN THIS CLASS

        Instead, fill in the BicycleConfigurationSpace at the bottom of the
        file which inherits from this class.
    """

    def __init__(self, dim, low_lims, high_lims, obstacles, dt=0.01):
        """
        Parameters
        ----------
        dim: dimension of the state space: number of state variables.
        low_lims: the lower bounds of the state variables. Should be an
                iterable of length dim.
        high_lims: the higher bounds of the state variables. Should be an
                iterable of length dim.
        obstacles: A list of obstacles. This could be in any representation
            we choose, based on the application. In this project, for the bicycle
            model, we assume each obstacle is a circle in x, y space, and then
            obstacles is a list of [x, y, r] lists specifying the center and 
            radius of each obstacle.
        dt: The discretization timestep our local planner should use when constructing
            plans.
        """
        self.dim = dim
        self.low_lims = np.array(low_lims)
        self.high_lims = np.array(high_lims)
        self.obstacles = obstacles
        self.dt = dt

    def distance(self, c1, c2):
        """
            Implements the chosen metric for this configuration space.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.

            Returns the distance between configurations c1 and c2 according to
            the chosen metric.
        """
        pass

    def sample_config(self, *args):
        """
            Samples a new configuration from this C-Space according to the
            chosen probability measure.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.

            Returns a new configuration sampled at random from the configuration
            space.
        """
        pass

    def check_collision(self, c):
        """
            Checks to see if the specified configuration c is in collision with
            any obstacles.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.
        """
        pass

    def check_path_collision(self, path):
        """
            Checks to see if a specified path through the configuration space is 
            in collision with any obstacles.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.
        """
        pass

    def local_plan(self, c1, c2):
        """
            Constructs a plan from configuration c1 to c2.

            This is the local planning step in RRT. This should be where you extend
            the trajectory of the robot a little bit starting from c1. This may not
            constitute finding a complete plan from c1 to c2. Remember that we only
            care about moving in some direction while respecting the kinemtics of
            the robot. You may perform this step by picking a number of motion
            primitives, and then returning the primitive that brings you closest
            to c2.
        """
        pass

    def nearest_config_to(self, config_list, config):
        """
            Finds the configuration from config_list that is closest to config.
        """
        return min(config_list, key=lambda c: self.distance(c, config))

class FreeEuclideanSpace(ConfigurationSpace):
    """
        Example implementation of a configuration space. This class implements
        a configuration space representing free n dimensional euclidean space.
    """

    def __init__(self, dim, low_lims, high_lims, sec_per_meter=4):
        super(FreeEuclideanSpace, self).__init__(dim, low_lims, high_lims, [])
        self.sec_per_meter = sec_per_meter

    def distance(self, c1, c2):
        """
        c1 and c2 should by numpy.ndarrays of size (dim, 1) or (1, dim) or (dim,).
        """
        return np.linalg.norm(c1 - c2)

    def sample_config(self, *args):
        return np.random.uniform(self.low_lims, self.high_lims).reshape((self.dim,))

    def check_collision(self, c):
        return False

    def check_path_collision(self, path):
        return False

    def local_plan(self, c1, c2):
        v = c2 - c1
        dist = np.linalg.norm(c1 - c2)
        total_time = dist * self.sec_per_meter
        vel = v / total_time
        p = lambda t: (1 - (t / total_time)) * c1 + (t / total_time) * c2
        times = np.arange(0, total_time, self.dt)
        positions = p(times[:, None])
        velocities = np.tile(vel, (positions.shape[0], 1))
        plan = Plan(times, positions, velocities, dt=self.dt)
        return plan

class BicycleConfigurationSpace(ConfigurationSpace):
    """
        The configuration space for a Bicycle modeled robot
        Obstacles should be tuples (x, y, r), representing circles of 
        radius r centered at (x, y)
        We assume that the robot is circular and has radius equal to robot_radius
        The state of the robot is defined as (x, y, theta, phi).
    """
    def __init__(self, low_lims, high_lims, input_low_lims, input_high_lims, obstacles, robot_radius):
        dim = 4
        super(BicycleConfigurationSpace, self).__init__(dim, low_lims, high_lims, obstacles)
        self.robot_radius = robot_radius
        self.robot_length = 0.3
        self.input_low_lims = input_low_lims
        self.input_high_lims = input_high_lims

    def distance(self, c1, c2):
        """
        c1 and c2 should be numpy.ndarrays of size (4,)
        """
        dx = c1[0] - c2[0]
        dy = c1[1] - c2[1]
        dtheta = np.abs(c1[2]-c2[2])
        dphi = np.abs(c1[3]-c2[3])
        return np.sqrt(dx**2 + dy**2 + dtheta**2 + dphi**2)

    def sample_config(self, *args):
        """
        Pick a random configuration from within our state boundaries.

        You can pass in any number of additional optional arguments if you
        would like to implement custom sampling heuristics. By default, the
        RRT implementation passes in the goal as an additional argument,
        which can be used to implement a goal-biasing heuristic.
        """
        x = np.random.uniform(self.low_lims[0], self.high_lims[0])
        y = np.random.uniform(self.low_lims[1], self.high_lims[1])
        theta = np.random.uniform(self.low_lims[2], self.high_lims[2])
        phi = np.random.uniform(self.low_lims[3], self.high_lims[3])
        return np.array([x,y,theta,phi])

    def check_collision(self, c):
        """
        Returns true if a configuration c is in collision
        c should be a numpy.ndarray of size (4,)
        """
        robot_x, robot_y = c[0], c[1]
        for obs in self.obstacles:
            obs_x, obs_y, obs_r = obs
            dist = np.sqrt((robot_x - obs_x)**2 + (robot_y-obs_y)**2)
            if dist < (self.robot_radius + obs_r):
                return True
        return False 

    def check_path_collision(self, path):
        """
        Returns true if the input path is in collision. The path
        is given as a Plan object. See configuration_space.py
        for details on the Plan interface.

        You should also ensure that the path does not exceed any state bounds,
        and the open loop inputs don't exceed input bounds.
        """
        for time, position, control_input in path:
            if self.check_collision(position):
                return True 
        return False 
            
    def build_motion_primitives(curr_orientation):
            motion_primitives = []
            # motion primitives given as a vector [x, y, theta, phi], represents a transformation in state space
            # normalized to 1 and should be multiplied by dt in path planning
            x, y, theta, phi = curr_orientation
            equal_weight = (1 / np.sqrt(2))        

            # drive forward
            forward = np.array([np.cos(theta + phi), np.sin(theta + phi), 0, 0])
            motion_primitives.append(forward)

            # drive backward
            backward = np.array([-1 * np.cos(theta + phi), -1 * np.sin(theta + phi), 0, 0])
            motion_primitives.append(backward)

            # turn left
            left = np.array([0, 0, 0, 1])
            motion_primitives.append(left)

            # turn right
            right = np.array([0, 0, 0, -1])
            motion_primitives.append(right)

            # drive forward and turn left
            forward_and_left = np.array([equal_weight * np.cos(theta + phi), equal_weight * np.sin(theta + phi), 0, equal_weight * 1])
            motion_primitives.append(forward_and_left)

            # drive forward and turn right
            forward_and_left = np.array([equal_weight * np.cos(theta + phi), equal_weight * np.sin(theta + phi), 0, equal_weight * -1])
            motion_primitives.append(forward_and_left)

            # drive backwards and turn left
            forward_and_left = np.array([-1 * equal_weight * np.cos(theta + phi), -1 * equal_weight * np.sin(theta + phi), 0, equal_weight * 1])
            motion_primitives.append(forward_and_left)

            # drive backwards and turn right
            forward_and_left = np.array([-1 * equal_weight * np.cos(theta + phi), -1 * equal_weight * np.sin(theta + phi), 0, equal_weight * -1])
            motion_primitives.append(forward_and_left)

            return motion_primitives

    def local_plan(self, c1, c2, dt=0.01):
        """
        Constructs a local plan from c1 to c2. Usually, you want to
        just come up with any plan without worrying about obstacles,
        because the algorithm checks to see if the path is in collision,
        in which case it is discarded.

        However, in the case of the nonholonomic bicycle model, it will
        be very difficult for you to come up with a complete plan from c1
        to c2. Instead, you should choose a set of "motion-primitives", and
        then simply return whichever motion primitive brings you closest to c2.

        A motion primitive is just some small, local motion, that we can perform
        starting at c1. If we keep a set of these, we can choose whichever one
        brings us closest to c2.

        Keep in mind that choosing this set of motion primitives is tricky.
        Every plan we come up with will just be a bunch of these motion primitives
        chained together, so in order to get a complete motion planner, you need to 
        ensure that your set of motion primitives is such that you can get from any
        point to any other point using those motions.

        For example, in cartesian space, a set of motion primitives could be 
        {a1*x, a2*y, a3*z} where a1*x means moving a1 units in the x direction and
        so on. By varying a1, a2, a3, we get our set of primitives, and as you can
        see this set of primitives is rich enough that we can, indeed, get from any
        point in cartesian space to any other point by chaining together a bunch
        of these primitives. Then, this local planner would just amount to picking 
        the values of a1, a2, a3 that bring us closest to c2.

        You should spend some time thinking about what motion primitives would
        be good to use for a bicycle model robot. What kinds of motions are at
        our disposal?

        This should return a cofiguration_space.Plan object.
        """
        x1, y1, theta1, phi1 = c1
        x2, y2, theta2, phi2 = c2 

        distance = np.linalg.norm([x2-x1, y2-y1]) # can use dist func
        total_time = distance #fix this 

        motion_primitives = self.build_motion_primitives(c1)
        min_dist = float("inf")
        min_index = 0

        for i in range(len(motion_primitives)):
            c1_new = c1 + motion_primitives[i]*dt
            curr_dist = self.distance(c1_new, c2)
            if curr_dist < min_dist:
                min_dist = curr_dist
                min_index = i
        
        return motion_primitives[min_index] # not right, need return plan object

        #Motion Primitives Pseudocode 

        # motion_primitives = build_motion_primitives(curr_orientation)

        # for motion in motion_primitives:
        #     update c1 with motion * dt
        #     check distance c1_new and c2
        #     min distance = motion_primitives

        # return plan object with right primitive
        plan = Plan(times, positions, open_loop_inputs, dt)
        return plan 
        

