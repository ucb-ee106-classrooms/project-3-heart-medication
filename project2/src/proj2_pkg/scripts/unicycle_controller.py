#!/usr/bin/env python
"""
Bicycle Model Controller - Project 2 Implementation
------------------------------------------------
This controller implements a trajectory tracking approach using a combined feedforward-feedback strategy.
It focuses on the practical implementation aspects rather than the pure theoretical approach from the paper.

Key Features:
1. Combined Feedforward-Feedback Control:
   - Uses planned trajectory inputs as feedforward terms
   - Adds PD control feedback for error correction
   - Separates control into position (x,y) and orientation (theta, phi) components

2. Error Handling:
   - Explicit handling of angular wrapping for theta and phi
   - Derivative error terms for smoother control
   - Numerical safeguards (e.g., minimum dt)

3. Integration with ROS:
   - Properly interfaces with bicycle model converter
   - Uses BicycleCommandMsg for control inputs
   - Handles state feedback through BicycleStateMsg

Control Strategy:
- Position Control: Uses proportional-derivative (PD) control on x,y errors
- Orientation Control: Separate PD control for theta and phi
- Combined Control: Merges planned inputs with feedback corrections

Differences from Paper Implementation:
1. Uses simpler PD control instead of full nonlinear control law
2. Focuses on practical trajectory tracking rather than theoretical guarantees
3. Adds robustness features for real-world implementation

Usage:
- This controller is designed to work with the planners through main.py
- It executes planned trajectories while providing feedback correction
- Gains can be tuned through kp and kd parameters
"""

import numpy as np
import rospy
from proj2_pkg.msg import BicycleCommandMsg, BicycleStateMsg 