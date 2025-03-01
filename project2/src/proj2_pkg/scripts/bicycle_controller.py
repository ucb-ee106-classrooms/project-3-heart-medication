#!/usr/bin/env python
"""
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

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos, sqrt 