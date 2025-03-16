#!/usr/bin/env python

"""
Starter code for EECS C106B Spring 2020 Project 2.
Author: Amay Saxena
"""
import numpy as np
import sys
<<<<<<< HEAD
import matplotlib.pyplot as plt

import tf.transformations
=======

>>>>>>> 2028cc461d4b5c6f30e8ad0d038d75c211a80652
import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv
import rospy
<<<<<<< HEAD
from geometry_msgs.msg import Twist 
=======
>>>>>>> 2028cc461d4b5c6f30e8ad0d038d75c211a80652
from proj2_pkg.msg import BicycleCommandMsg, BicycleStateMsg
from proj2.planners import SinusoidPlanner, RRTPlanner, BicycleConfigurationSpace

class BicycleModelController(object):
    def __init__(self):
        """
        Executes a plan made by the planner
        """
<<<<<<< HEAD
        
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe)
        # self.tfBuffer = tf2_ros.Buffer()
        # self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        # self.step_control_state = Twist()
=======
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe)
>>>>>>> 2028cc461d4b5c6f30e8ad0d038d75c211a80652
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
<<<<<<< HEAD

        actual_x_states, actual_y_states, actual_theta_states, actual_phi_states  = [], [], [], []
        desired_x_states, desired_y_states, desired_theta_states, desired_phi_states  = [], [], [], []
        time_data = []
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_t).to_sec()
            print(f"t is: {t} \n")
            if t > plan.times[-1]:
                break

            # trans_odom_to_base_link = self.tfBuffer.lookup_transform('base_footprint', 'odom', rospy.Time(), rospy.Duration(5))
            # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([trans_odom_to_base_link.transform.rotation.x, trans_odom_to_base_link.transform.rotation.y, trans_odom_to_base_link.transform.rotation.z, trans_odom_to_base_link.transform.rotation.w])
            # self.step_control_state = [trans_odom_to_base_link.transform.translation.x, trans_odom_to_base_link.transform.translation.y, yaw,0]
            state, cmd = plan.get(t)
            #breakpoint()
            self.step_control(state, cmd)

            #to plot it 
            time_data.append(t)
            x, y, theta, phi = self.state
            actual_x_states.append(x)
            actual_y_states.append(y)
            actual_theta_states.append(theta)
            actual_phi_states.append(phi)

            x_desired, y_desired, theta_desired, phi_desired = state
            desired_x_states.append(x_desired)
            desired_y_states.append(y_desired)
            desired_theta_states.append(theta_desired)
            desired_phi_states.append(phi_desired)


            print(f"state is: {state}")
            rate.sleep()
        
        self.cmd([0, 0])

        #matplotlib
        fig, axes = plt.subplots(4,1,figsize =(10,12), sharex=True)

        axes[0].plot(time_data, actual_x_states, label="actual x", linestyle='-', marker='o', color='b')
        axes[0].plot(time_data, desired_x_states, label="desired x", linestyle='-', marker='o', color='r')
        axes[0].set_ylabel("X position")
        axes[0].legend()
        axes[0].grid()

        axes[1].plot(time_data, actual_y_states, label="actual y", linestyle='-', marker='o', color='b')
        axes[1].plot(time_data, desired_y_states, label="desired y", linestyle='-', marker='o', color='r')
        axes[1].set_ylabel("Y position")
        axes[1].legend()
        axes[1].grid()

        axes[2].plot(time_data, actual_theta_states, label="actual theta", linestyle='-', marker='o', color='b')
        axes[2].plot(time_data, desired_theta_states, label="desired theta", linestyle='-', marker='o', color='r')
        axes[2].set_ylabel("theta position")
        axes[2].legend()
        axes[2].grid()

        axes[3].plot(time_data, actual_phi_states, label="actual phi", linestyle='-', marker='o', color='b')
        axes[3].plot(time_data, desired_phi_states, label="desired phi", linestyle='-', marker='o', color='r')
        axes[3].set_ylabel("phi position")
        axes[3].legend()
        axes[3].grid()



        plt.tight_layout()
        plt.show()
        # self.state = state
        #breakpoint()

=======
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_t).to_sec()
            if t > plan.times[-1]:
                break
            state, cmd = plan.get(t)
            self.step_control(state, cmd)
            rate.sleep()
        self.cmd([0, 0])

>>>>>>> 2028cc461d4b5c6f30e8ad0d038d75c211a80652
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
<<<<<<< HEAD
        
        ### OPEN LOOP CONTROL ###
        # print(f"open_loop_input: {open_loop_input}")
        
        # self.cmd(open_loop_input)
        
        
        ## PD CONTROLLER ###
        # Kp = [1.0, 1.0, 1.5, 1.5]
        # Kd = [0.1,0.1,0.2,0.2]
        # # current_position = self.state 
        # current_position = self.state
        # x, y, theta, phi = current_position
        
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


        ### LYAPUNOV BASED DESIGN [16], page 20
        k1, k2, k3 = 0.2, 0.5, 0.1
        x, y, theta, phi = self.state
        x_ref, y_ref, theta_ref, phi_ref = target_position
        v_ref, w_ref = open_loop_input
        R = np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 0], [0,0,1]])
        delta = np.array([x_ref-x, y_ref-y, theta_ref-theta])

        x_e, y_e, theta_e = R @ delta
        v_r = v_ref*np.cos(theta_e)+k1*x_e
        w = w_ref+v_ref*(k2*y_e + k3*np.sin(theta_e))
        self.cmd(np.array([v_r, w]))


=======
        self.cmd(open_loop_input)
>>>>>>> 2028cc461d4b5c6f30e8ad0d038d75c211a80652


    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : numpy.ndarray
        """
<<<<<<< HEAD
        ## CHANGING MSG TYPE TO TWIST TO WORK ON THE TURTLEBOT
        # cmd = Twist()
        # cmd.linear.x = msg[0]
        # cmd.angular.z = msg[1]
        # self.pub.publish(cmd)
=======
>>>>>>> 2028cc461d4b5c6f30e8ad0d038d75c211a80652
        self.pub.publish(BicycleCommandMsg(*msg))

    def subscribe(self, msg):
        """
        callback fn for state listener.  Don't call me...
        
        Parameters
        ----------
        msg : :obj:`BicycleStateMsg`
        """
<<<<<<< HEAD
        # print("state is", self.state)
        #ISSUE: Initial State is [0,0,0,0]
        # self.state = np.array([1.0, 1.0, 0.0, 0.0])
=======
>>>>>>> 2028cc461d4b5c6f30e8ad0d038d75c211a80652
        self.state = np.array([msg.x, msg.y, msg.theta, msg.phi])

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.cmd((0, 0))
