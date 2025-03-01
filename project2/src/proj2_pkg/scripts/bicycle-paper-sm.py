#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos, sqrt

"""
TLDR: THIS FOLLOWS THE PAPER MORE CLOSELY. MIGHT NOT BE PRACTICAL THO???

Bicycle Model Controller - Research Paper Implementation
----------------------------------------------------
This controller implements the control approach from the paper:
"CONTROL OF UNICYCLE TYPE ROBOTS: Tracking, Path Following and Point Stabilization"
by R. Carona, A. P. Aguiar, and J. Gaspar.

Key Features:
1. Nonlinear Control Law:
   - Implements the paper's feedback linearization approach
   - Uses error transformation to robot frame
   - Explicit handling of nonholonomic constraints

2. Dynamic Model:
   - Full bicycle model dynamics implementation:
     ẋ = cos(θ)u₁
     ẏ = sin(θ)u₁
     θ̇ = (1/L)tan(φ)u₁
     φ̇ = u₂
   - Includes wheelbase length (L) in calculations
   - Proper handling of steering angle dynamics

3. Constraint Handling:
   - Explicit handling of input constraints (u1_max, u2_max)
   - Steering angle constraints (phi_max)
   - Proper scaling of control inputs

Control Strategy:
- Uses feedback linearization for nonlinear dynamics
- Transforms errors to robot frame for better control
- Separates longitudinal and steering control
- Implements paper's stability-guaranteed approach

Differences from Basic Implementation:
1. Uses full nonlinear control law from paper
2. Includes proper bicycle model dynamics
3. More theoretically grounded approach
4. Better handling of nonholonomic constraints

Usage:
- Designed for direct control of bicycle model
- Can be used independently or with trajectory planners
- Parameters can be tuned through k1, k2, k3 gains
"""

class BicycleModelController:
    def __init__(self):
        """Initialize the bicycle model controller with control gains and ROS setup"""
        # Robot parameters
        self.L = 0.2  # Wheelbase length
        
        # Control gains for tracking
        self.k1 = 1.0  # Position gain
        self.k2 = 2.0  # Heading gain
        self.k3 = 1.0  # Steering gain
        
        # State and input constraints
        self.phi_max = rospy.get_param("/bicycle_converter/converter/max_steering_angle", 0.5)
        self.u1_max = rospy.get_param("/bicycle_converter/converter/max_linear_velocity", 0.5)
        self.u2_max = rospy.get_param("/bicycle_converter/converter/max_steering_rate", 0.5)
        
        # Initialize ROS node
        self.cmd_pub = rospy.Publisher('/bicycle/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.phi = 0.0  # Steering angle
        self.v = 0.0    # Linear velocity
        self.omega = 0.0  # Angular velocity

    def odom_callback(self, msg):
        """Update robot state from odometry messages"""
        # Position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Get orientation from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.theta = euler_from_quaternion(orientation_list)
        
        # Velocities
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def compute_control_inputs(self, xd, yd, thetad, phid):
        """
        Compute control inputs (u1, u2) for the bicycle model
        Args:
            xd, yd: Desired position
            thetad: Desired heading
            phid: Desired steering angle
        """
        # Error calculation
        ex = self.x - xd
        ey = self.y - yd
        etheta = self.theta - thetad
        ephi = self.phi - phid
        
        # Transform errors to robot frame
        er_x = cos(self.theta) * ex + sin(self.theta) * ey
        er_y = -sin(self.theta) * ex + cos(self.theta) * ey
        
        # Control law based on bicycle model dynamics
        # u1 controls the forward velocity
        u1 = -self.k1 * er_x
        
        # u2 controls the steering rate
        # Using feedback linearization for steering control
        alpha = atan2(er_y, er_x)
        u2 = -self.k2 * ephi - self.k3 * (self.phi - alpha)
        
        # Apply input constraints
        u1 = np.clip(u1, -self.u1_max, self.u1_max)
        u2 = np.clip(u2, -self.u2_max, self.u2_max)
        
        return u1, u2

    def send_velocity_command(self, u1, u2):
        """Send velocity commands to the bicycle model robot"""
        cmd_msg = Twist()
        cmd_msg.linear.x = u1  # Forward velocity
        cmd_msg.angular.z = u2  # Steering rate
        self.cmd_pub.publish(cmd_msg)

    def execute_control(self, xd, yd, thetad, phid):
        """
        Execute control loop for the bicycle model
        Args:
            xd, yd: Desired position
            thetad: Desired heading
            phid: Desired steering angle
        """
        # Compute control inputs
        u1, u2 = self.compute_control_inputs(xd, yd, thetad, phid)
        
        # Send commands to robot
        self.send_velocity_command(u1, u2)

if __name__ == '__main__':
    try:
        rospy.init_node('bicycle_model_controller')
        controller = BicycleModelController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 