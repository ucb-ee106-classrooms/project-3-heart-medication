#!/usr/bin/env python

"""
Starter code for EECS C106B Spring 2020 Project 2.
Author: Amay Saxena
"""
import numpy as np
import sys

import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv
import rospy
from proj2_pkg.msg import BicycleCommandMsg, BicycleStateMsg
from proj2.planners import SinusoidPlanner, RRTPlanner, BicycleConfigurationSpace

class BicycleModelController(object):
    def __init__(self):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe)
        self.state = BicycleStateMsg()
        rospy.on_shutdown(self.shutdown)

    def execute_plan(self, plan):
        """
        Executes a plan made by the planner

        Parameters
        ----------
        plan : :obj: Plan. See configuration_space.Plan
        """
        if len(plan) == 0:
            return
        rate = rospy.Rate(int(1 / plan.dt))
        start_t = rospy.Time.now()
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_t).to_sec()
            if t > plan.times[-1]:
                break
            state, cmd = plan.get(t)
            #breakpoint()
            self.step_control(state, cmd)
            rate.sleep()
        self.cmd([0, 0])

    def step_control(self, target_position, open_loop_input):
        """Specify a control law. For the grad/EC portion, you may want
        to edit this part to write your own closed loop controller.
        Note that this class constantly subscribes to the state of the robot,
        so the current configuratin of the robot is always stored in the 
        variable self.state. You can use this as your state measurement
        when writing your closed loop controller.

        Parameters
        ----------
            target_position : target position at the current step in
                              [x, y, theta, phi] configuration space.
            open_loop_input : the prescribed open loop input at the current
                              step, as a [u1, u2] pair.
        Returns:
            None. It simply sends the computed command to the robot.
        """
        #print(f"open_loop_input: {open_loop_input}")
        
        self.cmd(open_loop_input)
        
        
        ### PD CONTROLLER ###
        # Kp = [1.0, 1.0, 0.5, 0.5]
        # Kd = [0.1,0.1,0.05,0.05]
        # current_position = self.state 
        # x, y, theta, phi = self.state
        
        # error = [target_position[i] - current_position[i] for i in range(4)]
        # current_time = rospy.Time.now().to_sec()

        # if hasattr(self, "prev_time"):
        #     dt = current_time - self.prev_time 
        #     dt = max(dt, 1e-6)
        # else:
        #     dt = 1e-3
        
        # if hasattr(self, "prev_error"):
        #     error_derivative = [(error[i]-self.prev_error[i])/dt for i in range(4)]
        # else:
        #     error_derivative = [0.0]*4
        # x_dot, y_dot, theta_dot, phi_dot = [Kp[i]*error[i]+Kd[i]*error_derivative[i] for i in range(4)]
        # u2 = phi_dot
        # if np.cos(theta) != 0:
        #     u1 = x_dot / np.cos(theta)
        # else:
        #     u1 = y_dot / np.sin(theta)
        # #u1_3 = (theta_dot * self.l
        # control_input = np.array([u1, u2])

        # self.cmd(control_input)
        # self.prev_error = error 
        # self.prev_time = current_time 
        



    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : numpy.ndarray
        """
        self.pub.publish(BicycleCommandMsg(*msg))

    def subscribe(self, msg):
        """
        callback fn for state listener.  Don't call me...
        
        Parameters
        ----------
        msg : :obj:`BicycleStateMsg`
        """
        # print("state is", self.state)
        #ISSUE: Initial State is [0,0,0,0]
        # self.state = np.array([1.0, 1.0, 0.0, 0.0])
        self.state = np.array([msg.x, msg.y, msg.theta, msg.phi])

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.cmd((0, 0))
