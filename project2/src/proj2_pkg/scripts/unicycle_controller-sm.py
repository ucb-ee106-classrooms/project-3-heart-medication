#!/usr/bin/env python

import numpy as np
import rospy
from proj2_pkg.msg import BicycleCommandMsg, BicycleStateMsg

class BicycleModelController(object):
    def __init__(self):
        """
        Initialize the bicycle model controller
        """
        # Control gains
        self.kp = [1.0, 1.0, 0.5, 0.5]  # Position, steering gains
        self.kd = [0.1, 0.1, 0.05, 0.05]  # Derivative gains
        
        # Initialize ROS node
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe)
        self.state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, theta, phi]
        
        # For derivative control
        self.prev_error = None
        self.prev_time = None
        
        rospy.on_shutdown(self.shutdown)

    def execute_plan(self, plan):
        """
        Executes a plan made by the planner
        Parameters
        ----------
        plan : Plan object with trajectory and inputs
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
            self.step_control(state, cmd)
            rate.sleep()
            
        self.cmd([0, 0])  # Stop at end of trajectory

    def step_control(self, target_position, open_loop_input):
        """
        Implements closed-loop control for trajectory tracking
        Parameters
        ----------
        target_position : [x, y, theta, phi] target configuration
        open_loop_input : [u1, u2] open loop inputs from planner
        """
        # Calculate position error
        error = target_position - self.state
        
        # Normalize angular errors to [-pi, pi]
        error[2] = (error[2] + np.pi) % (2 * np.pi) - np.pi  # theta error
        error[3] = (error[3] + np.pi) % (2 * np.pi) - np.pi  # phi error
        
        # Calculate derivative terms
        current_time = rospy.Time.now().to_sec()
        if self.prev_time is not None:
            dt = current_time - self.prev_time
            dt = max(dt, 1e-6)  # Prevent division by zero
            error_derivative = (error - self.prev_error) / dt
        else:
            error_derivative = np.zeros(4)
        
        # Compute control input (PD control + feedforward)
        control_input = np.zeros(2)
        
        # Forward velocity (u1) control
        control_input[0] = (self.kp[0] * error[0] + self.kd[0] * error_derivative[0] + 
                          self.kp[1] * error[1] + self.kd[1] * error_derivative[1] +
                          open_loop_input[0])
                          
        # Steering rate (u2) control
        control_input[1] = (self.kp[2] * error[2] + self.kd[2] * error_derivative[2] +
                          self.kp[3] * error[3] + self.kd[3] * error_derivative[3] +
                          open_loop_input[1])
        
        # Store values for next iteration
        self.prev_error = error
        self.prev_time = current_time
        
        # Send command to robot
        self.cmd(control_input)

    def cmd(self, msg):
        """
        Sends a command to the robot
        Parameters
        ----------
        msg : [u1, u2] control inputs
        """
        self.pub.publish(BicycleCommandMsg(*msg))

    def subscribe(self, msg):
        """
        Callback for state updates
        Parameters
        ----------
        msg : BicycleStateMsg
        """
        self.state = np.array([msg.x, msg.y, msg.theta, msg.phi])

    def shutdown(self):
        """
        Clean shutdown of the controller
        """
        rospy.loginfo("Shutting Down Controller")
        self.cmd([0, 0])

if __name__ == '__main__':
    try:
        rospy.init_node('bicycle_model_controller')
        controller = BicycleModelController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 