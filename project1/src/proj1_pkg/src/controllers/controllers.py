#!/usr/bin/env python

"""
Starter script for Project 1B. 
Author: Chris Correa, Valmik Prabhu
"""

# Python imports
import sys
import numpy as np
import itertools
import matplotlib
import keyboard
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.linalg import logm

# Lab imports
from utils.utils import *

# ROS imports
try:
    import tf
    import tf2_ros
    import rospy
    import baxter_interface
    import intera_interface
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotTrajectory
except:
    pass

NUM_JOINTS = 7

class Controller:

    def __init__(self, limb, kin):
        """
        Constructor for the superclass. All subclasses should call the superconstructor

        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        """

        # Run the shutdown function when the ros node is shutdown
        rospy.on_shutdown(self.shutdown)
        self._limb = limb
        self._kin = kin

        # Set this attribute to True if the present controller is a jointspace controller.
        self.is_jointspace_controller = False

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly.  

        Parameters
        ----------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        """
        pass

    def interpolate_path(self, path, t, current_index = 0):
        """
        interpolates over a :obj:`moveit_msgs.msg.RobotTrajectory` to produce desired
        positions, velocities, and accelerations at a specified time

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        t : float
            the time from start
        current_index : int
            waypoint index from which to start search

        Returns
        -------
        target_position : 7x' or 6x' :obj:`numpy.ndarray` 
            desired positions
        target_velocity : 7x' or 6x' :obj:`numpy.ndarray` 
            desired velocities
        target_acceleration : 7x' or 6x' :obj:`numpy.ndarray` 
            desired accelerations
        current_index : int
            waypoint index at which search was terminated 
        """

        # a very small number (should be much smaller than rate)
        epsilon = 0.0001

        max_index = len(path.joint_trajectory.points)-1

        # If the time at current index is greater than the current time,
        # start looking from the beginning
        if (path.joint_trajectory.points[current_index].time_from_start.to_sec() > t):
            current_index = 0

        # Iterate forwards so that you're using the latest time
        while (
            not rospy.is_shutdown() and \
            current_index < max_index and \
            path.joint_trajectory.points[current_index+1].time_from_start.to_sec() < t+epsilon
        ):
            current_index = current_index+1

        # Perform the interpolation
        if current_index < max_index:
            time_low = path.joint_trajectory.points[current_index].time_from_start.to_sec()
            time_high = path.joint_trajectory.points[current_index+1].time_from_start.to_sec()

            target_position_low = np.array(
                path.joint_trajectory.points[current_index].positions
            )
            target_velocity_low = np.array(
                path.joint_trajectory.points[current_index].velocities
            )
            target_acceleration_low = np.array(
                path.joint_trajectory.points[current_index].accelerations
            )

            target_position_high = np.array(
                path.joint_trajectory.points[current_index+1].positions
            )
            target_velocity_high = np.array(
                path.joint_trajectory.points[current_index+1].velocities
            )
            target_acceleration_high = np.array(
                path.joint_trajectory.points[current_index+1].accelerations
            )

            target_position = target_position_low + \
                (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + \
                (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)
            target_acceleration = target_acceleration_low + \
                (t - time_low)/(time_high - time_low)*(target_acceleration_high - target_acceleration_low)

        # If you're at the last waypoint, no interpolation is needed
        else:
            target_position = np.array(path.joint_trajectory.points[current_index].positions)
            target_velocity = np.array(path.joint_trajectory.points[current_index].velocities)
            target_acceleration = np.array(path.joint_trajectory.points[current_index].velocities)

        return (target_position, target_velocity, target_acceleration, current_index)


    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self.stop_moving()
        rospy.sleep(0.1)

    def stop_moving(self):
        """
        Set robot joint velocities to zero
        """
        zero_vel_dict = joint_array_to_dict(np.zeros(NUM_JOINTS), self._limb)
        self._limb.set_joint_velocities(zero_vel_dict)

    def plot_results(
        self,
        times,
        actual_positions, 
        actual_velocities, 
        target_positions, 
        target_velocities
    ):
        """
        Plots results.
        If the path is in joint space, it will plot both workspace and jointspace plots.
        Otherwise it'll plot only workspace

        Inputs:
        times : nx' :obj:`numpy.ndarray`
        actual_positions : nx7 or nx6 :obj:`numpy.ndarray`
            actual joint positions for each time in times
        actual_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            actual joint velocities for each time in times
        target_positions: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace positions for each time in times
        target_velocities: nx7 or nx6 :obj:`numpy.ndarray`
            target joint or workspace velocities for each time in times
        """

        # Make everything an ndarray
        times = np.array(times)
        actual_positions = np.array(actual_positions)
        actual_velocities = np.array(actual_velocities)
        target_positions = np.array(target_positions)
        target_velocities = np.array(target_velocities)

        # Find the actual workspace positions and velocities
        actual_workspace_positions = np.zeros((len(times), 3))
        actual_workspace_velocities = np.zeros((len(times), 3))
        actual_workspace_quaternions = np.zeros((len(times), 4))

        for i in range(len(times)):
            positions_dict = joint_array_to_dict(actual_positions[i], self._limb)
            fk = self._kin.forward_position_kinematics(joint_values=positions_dict)
            
            actual_workspace_positions[i, :] = fk[:3]
            actual_workspace_velocities[i, :] = \
                self._kin.jacobian(joint_values=positions_dict)[:3].dot(actual_velocities[i])
            actual_workspace_quaternions[i, :] = fk[3:]
        # check if joint space
        if self.is_jointspace_controller:
            # it's joint space

            target_workspace_positions = np.zeros((len(times), 3))
            target_workspace_velocities = np.zeros((len(times), 3))
            target_workspace_quaternions = np.zeros((len(times), 4))

            for i in range(len(times)):
                positions_dict = joint_array_to_dict(target_positions[i], self._limb)
                target_workspace_positions[i, :] = \
                    self._kin.forward_position_kinematics(joint_values=positions_dict)[:3]
                target_workspace_velocities[i, :] = \
                    self._kin.jacobian(joint_values=positions_dict)[:3].dot(target_velocities[i])
                target_workspace_quaternions[i, :] = np.array([0, 1, 0, 0])

            # Plot joint space
            plt.figure()
            joint_num = len(self._limb.joint_names())
            for joint in range(joint_num):
                plt.subplot(joint_num,2,2*joint+1)
                plt.plot(times, actual_positions[:,joint], label='Actual')
                plt.plot(times, target_positions[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Position Error")
                plt.legend()

                plt.subplot(joint_num,2,2*joint+2)
                plt.plot(times, actual_velocities[:,joint], label='Actual')
                plt.plot(times, target_velocities[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                plt.ylabel("Joint " + str(joint) + " Velocity Error")
                plt.legend()
            print("Close the plot window to continue")
            plt.show()

        else:
            # it's workspace
            target_workspace_positions = target_positions
            target_workspace_velocities = target_velocities
            target_workspace_quaternions = np.zeros((len(times), 4))
            target_workspace_quaternions[:, 1] = 1

        plt.figure()
        workspace_joints = ('X', 'Y', 'Z')
        joint_num = len(workspace_joints)
        for joint in range(joint_num):
            plt.subplot(joint_num,2,2*joint+1)
            plt.plot(times, actual_workspace_positions[:,joint], label='Actual')
            plt.plot(times, target_workspace_positions[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Position Error")
            plt.legend()

            plt.subplot(joint_num,2,2*joint+2)
            plt.plot(times, actual_velocities[:,joint], label='Actual')
            plt.plot(times, target_velocities[:,joint], label='Desired')
            plt.xlabel("Time (t)")
            plt.ylabel(workspace_joints[joint] + " Velocity Error")
            plt.legend()

        print("Close the plot window to continue")
        plt.show()

        # Plot orientation error. This is measured by considering the
        # axis angle representation of the rotation matrix mapping
        # the desired orientation to the actual orientation. We use
        # the corresponding angle as our metric. Note that perfect tracking
        # would mean that this "angle error" is always zero.
        angles = []
        for t in range(len(times)):
            quat1 = target_workspace_quaternions[t]
            quat2 = actual_workspace_quaternions[t]
            theta = axis_angle(quat1, quat2)
            angles.append(theta)

        plt.figure()
        plt.plot(times, angles)
        plt.xlabel("Time (s)")
        plt.ylabel("Error Angle of End Effector (rad)")
        print("Close the plot window to continue")
        plt.show()
        

    def execute_path(self, path, rate=200, timeout=None, log=False):
        """
        takes in a path and moves the baxter in order to follow the path.  

        Parameters
        ----------
        path : :obj:`moveit_msgs.msg.RobotTrajectory`
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """

        # For plotting
        if log:
            times = list()
            actual_positions = list()
            actual_velocities = list()
            target_positions = list()
            target_velocities = list()

        # For interpolation
        max_index = len(path.joint_trajectory.points)-1
        current_index = 0

        # For timing
        start_t = rospy.Time.now()
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - start_t).to_sec()

            # If the controller has timed out, stop moving and return false
            if timeout is not None and t >= timeout:
                # Set velocities to zero
                self.stop_moving()
                return False

            current_position = get_joint_positions(self._limb)
            current_velocity = get_joint_velocities(self._limb)

            # Get the desired position, velocity, and effort
            (
                target_position, 
                target_velocity, 
                target_acceleration, 
                current_index
            ) = self.interpolate_path(path, t, current_index)

            # For plotting
            if log:
                times.append(t)
                actual_positions.append(current_position)
                actual_velocities.append(current_velocity)
                target_positions.append(target_position)
                target_velocities.append(target_velocity)

            # Run controller
            self.step_control(target_position, target_velocity, target_acceleration)

            # Sleep for a bit (to let robot move)
            r.sleep()

            if current_index >= max_index:
                self.stop_moving()
                break

        if log:
            self.plot_results(
                times,
                actual_positions, 
                actual_velocities, 
                target_positions, 
                target_velocities
            )
        return True

    def follow_ar_tag(self, tag, rate=200, timeout=None, log=False):
        """
        takes in an AR tag number and follows it with the baxter's arm.  You 
        should look at execute_path() for inspiration on how to write this. 

        Parameters
        ----------
        tag : int
            which AR tag to use
        rate : int
            This specifies how many ms between loops.  It is important to
            use a rate and not a regular while loop because you want the
            loop to refresh at a constant rate, otherwise you would have to
            tune your PD parameters if the loop runs slower / faster
        timeout : int
            If you want the controller to terminate after a certain number
            of seconds, specify a timeout in seconds.
        log : bool
            whether or not to display a plot of the controller performance

        Returns
        -------
        bool
            whether the controller completes the path or not
        """
        #need to convert to jointspace from workspace if we are using the jointspace controller!
        r = rospy.Rate(rate)
        tfBuffer = tf2_ros.Buffer()
        listener = tf.TransformListener()
        try:
            print('tag is', tag)
            listener.waitForTransform("base", f"ar_marker_{tag}", rospy.Time(0), rospy.Duration(3.0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get the transform: {e}")
        
        if log:
            times = []
            actual_positions = []
            actual_velocities = []
            target_positions = []
            target_velocities = []
        
        start_t = rospy.Time.now()
        timer = rospy.Time.now()

        while not rospy.is_shutdown() and timer.secs - start_t.secs < 40:
            #find time from start
            t = (rospy.Time.now()-start_t).to_sec()

            #if controller timed out, stop moving and return false 
            if timeout is not None and t>=timeout:
                self.stop_moving()
                return False

            try:
                #lookup transform for the ar tag
                now = rospy.Time(0)
                listener.waitForTransform("base", f"ar_marker_{tag}", now, rospy.Duration(1.0))
                (trans, rot) = listener.lookupTransform("base",f"ar_marker_{tag}", now)
                
                
                target_position = np.array(list(trans)+list(rot))
                
                
                #Attempt 1 - hardcoded target velocity
                # target_velocity = np.array([0.05,0.05,0.05,0.05,0.05,0.05,0.05]) #hardcode for now, later in terms of distance bw tag and sawyer
                
                #Attempt 2
                #get current end effector position
                print(get_joint_positions(self._limb))
                #current_end_effector_pose = self._kin.forward_position_kinematics(get_joint_positions(self._limb))[:3]
                
                #current_end_effector_pose = (get_joint_positions(self._limb))[:3] # this does not error but incorrect
                
                # Attempt 3
                # how to compare apples to apples? We have lookup transform for the AR tag, need joint_pos? Or fwd_poz_kine(get_joint_pos?) lookup transform
                # get_joint_pos = returns joint angles.
                # copy from the above AR tag code, "right_hand" from current_position in main
                # returns transform between base and right_hand
                arm_now = rospy.Time(0)
                listener.waitForTransform("base", f"right_hand", arm_now, rospy.Duration(1.0))
                (arm_trans, arm_rot) = listener.lookupTransform("base",f"right_hand", arm_now)
                
                # # Attempt 4: copy from main current_position
                # arm_trans = tfBuffer.lookup_transform("base", 'right_hand', rospy.Time(0), rospy.Duration(1.0))
    
                # current_position = np.array([getattr(arm_trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
                # print("Current Position:", current_position)


                #error between the current position and the ar tag position 
                position_error = np.array(trans)-np.array(arm_trans)
                position_error[2] = 0 #no vertical displacement, only side to side 

                #proportional control to set target velocity based on position error 
                Kp = 0.2
                
                # velo calc wrong; taking x, y, z pos error and sending it as velocity to 2 of 7 joints
                # how convert x, y translation to target_velocity in jointspace?
                # if get x, y translation to be in jointspace, then ok
                # how move workpace to jointspace
                # wait -- just do transform from ar_marker to right_hand? nah then get trans + rot
                # inverse kinematics?
                linear_velocity = Kp*position_error
                target_velocity = np.hstack((linear_velocity, np.zeros(3)))
                            
                # very questionable
                target_acceleration = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                safe_target_position = target_position.copy()
                safe_target_position[2] += 0.8 # keep z const
                # change target rotation to have hand straight down (see Trajectories.py:target_pose)
                safe_target_position[3], safe_target_position[4], safe_target_position[5], safe_target_position[6] = 0, 1, 0, 0 
                #breakpoint()                

                if log:
                    times.append(t)
                    actual_positions.append(get_joint_positions(self._limb))
                    actual_velocities.append(get_joint_velocities(self._limb))
                    target_positions.append(safe_target_position)
                    target_velocities.append(target_velocity)
                #step control based on the retrieved target pos/velocity
                self.step_control(safe_target_position, target_velocity, target_acceleration)
                timer = rospy.Time.now()

            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(f"Transform error: {e}")
            r.sleep()
        if log:
            self.plot_results(
                times,
                actual_positions,
                actual_velocities,
                target_positions,
                target_velocities
            )
        return True 


class FeedforwardJointVelocityController(Controller):
    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Parameters
        ----------
        target_position: 7x' ndarray of desired positions
        target_velocity: 7x' ndarray of desired velocities
        target_acceleration: 7x' ndarray of desired accelerations
        """
        self._limb.set_joint_velocities(joint_array_to_dict(target_velocity, self._limb))

class WorkspaceVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the
    PDJointVelocityController is that this controller compares the baxter's current WORKSPACE position and
    velocity desired WORKSPACE position and velocity to come up with a WORKSPACE velocity command to be sent
    to the baxter.  Then this controller should convert that WORKSPACE velocity command into a joint velocity
    command and sends that to the baxter.  Notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 6x' :obj:`numpy.ndarray`
        Kv : 6x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)
        self.is_jointspace_controller = False
        self.previous_desired = None


    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Makes a call to the robot to move according to its current position and the desired position 
        according to the input path and the current time.
        target_position will be a 7 vector describing the desired SE(3) configuration where the first
        3 entries are the desired position vector and the next 4 entries are the desired orientation as
        a quaternion, all written in spatial coordinates.
        target_velocity is the body-frame se(3) velocity of the desired SE(3) trajectory gd(t). This velocity
        is given as a 6D Twist (vx, vy, vz, wx, wy, wz).
        This method should call self._kin.forward_position_kinematics() to get the current workspace 
        configuration and self._limb.set_joint_velocities() to set the joint velocity to something.  
        Remember that we want to track a trajectory in SE(3), and implement the controller described in the
        project document PDF.
        Parameters
        ----------
        target_position: (7,) ndarray of desired SE(3) position (px, py, pz, qx, qy, qz, qw) (position + quaternion).
        target_velocity: (6,) ndarray of desired body-frame se(3) velocity (vx, vy, vz, wx, wy, wz).
        target_acceleration: ndarray of desired accelerations (should you need this?).
        """
        # raise NotImplementedError
        # control_input = None        
        # self._limb.set_joint_velocities(joint_array_to_dict(control_input, self._limb))
        spatial_twist = np.zeros(6)
        if self.previous_desired is not None:
            workspace = self._kin.forward_position_kinematics()
            g_target = get_g_matrix(self.previous_desired[:3], self.previous_desired[3:])
            g_tool = get_g_matrix(workspace[:3], workspace[3:])
            g_error = np.linalg.inv(g_tool) @ g_target
            g_hat = logm(g_error)
            twist = np.array([g_hat[0][3], g_hat[1][3], g_hat[2][3], g_hat[2][1], g_hat[0][2], g_hat[1][0]])
            spatial_twist = adj(g_tool)@twist
        u = self.Kp @ spatial_twist + target_velocity 
        print(target_velocity)
        angles = self._limb.joint_angles()
        control_input = self._kin.jacobian_pseudo_inverse(angles) @ u
        control_input = np.array(control_input.tolist())
        self._limb.set_joint_velocities(joint_array_to_dict(control_input[0], self._limb))

        # Update the previous desired position for the next control step
        self.previous_desired = target_position
        


class PDJointVelocityController(Controller):
    """
    Look at the comments on the Controller class above.  The difference between this controller and the 
    PDJointVelocityController is that this controller turns the desired workspace position and velocity
    into desired JOINT position and velocity.  Then it compares the difference between the baxter's 
    current JOINT position and velocity and desired JOINT position and velocity to come up with a
    joint velocity command and sends that to the baxter.  notice the shape of Kp and Kv
    """
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)
        self.is_jointspace_controller = True

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. his method should call
        get_joint_positions and get_joint_velocities from the utils package to get the current joint 
        position and velocity and self._limb.set_joint_velocities() to set the joint velocity to something.  
        You may find joint_array_to_dict() in utils.py useful as well.

        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        # raise NotImplementedError
        # control_input = None
        # self._limb.set_joint_velocities(joint_array_to_dict(control_input, self._limb))
        current_position = get_joint_positions(self._limb)
        current_velocity = get_joint_velocities(self._limb)

        position_error = target_position - current_position
        velocity_error = target_velocity - current_velocity

        control_input = np.dot(self.Kp, position_error)+np.dot(self.Kv, velocity_error)

        #set joint velocities
        self._limb.set_joint_velocities(joint_array_to_dict(control_input, self._limb))

class PDJointTorqueController(Controller):
    def __init__(self, limb, kin, Kp, Kv):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb`
        kin : :obj:`BaxterKinematics`
        Kp : 7x' :obj:`numpy.ndarray`
        Kv : 7x' :obj:`numpy.ndarray`
        """
        Controller.__init__(self, limb, kin)
        self.Kp = np.diag(Kp)
        self.Kv = np.diag(Kv)
        self.is_jointspace_controller = True

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Makes a call to the robot to move according to its current position and the desired position 
        according to the input path and the current time. This method should call
        get_joint_positions and get_joint_velocities from the utils package to get the current joint 
        position and velocity and self._limb.set_joint_torques() to set the joint torques to something. 
        You may find joint_array_to_dict() in utils.py useful as well.
        Recall that in order to implement a torque based controller you will need access to the 
        dynamics matrices M, C, G such that
        M ddq + C dq + G = u
        For this project, you will access the inertia matrix and gravity vector as follows:
        Inertia matrix: self._kin.inertia(positions_dict)
        Coriolis matrix: self._kin.coriolis(positions_dict, velocity_dict)
        Gravity matrix: self._kin.gravity(positions_dict) (You might want to scale this matrix by 0.01 or another scalar)
        These matrices were computed by a library and the coriolis matrix is approximate, 
        so you should think about what this means for the kinds of trajectories this 
        controller will be able to successfully track.
        Look in section 4.5 of MLS.
        Parameters
        ----------
        target_position: 7x' :obj:`numpy.ndarray` of desired positions
        target_velocity: 7x' :obj:`numpy.ndarray` of desired velocities
        target_acceleration: 7x' :obj:`numpy.ndarray` of desired accelerations
        """
        # raise NotImplementedError
        # control_input = None
        # self._limb.set_joint_torques(joint_array_to_dict(control_input, self._limb))

        positions_dict = joint_array_to_dict(get_joint_positions(self._limb), self._limb)
        velocity_dict = joint_array_to_dict(get_joint_velocities(self._limb), self._limb)

        position_error = target_position - get_joint_positions(self._limb)
        velocity_error = target_velocity - get_joint_velocities(self._limb)

        #dynamic matrices 
        M = self._kin.inertia(positions_dict)
        C = self._kin.coriolis(positions_dict, velocity_dict)
        G = self._kin.gravity(positions_dict)*0.01 

        #pd control law 
        desired_acceleration = np.dot(self.Kp, position_error) + np.dot(self.Kv, velocity_error) + target_acceleration
        control_input = np.dot(M, desired_acceleration) + C + G 

        self._limb.set_joint_torques(joint_array_to_dict(control_input, self._limb))


