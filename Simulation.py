import numpy as np
import mujoco
import mujoco_py as mj

import copy
import glfw
import imageio
import time
import sys

from mujoco_py import MjViewer
import mujoco_py.mjviewer


import robosuite
import robosuite as suite
import robosuite.utils.transform_utils as T
from robosuite.devices import *
from robosuite.models.robots import *
from robosuite.robots import *
from robosuite.controllers.joint_vel import JointVelocityController
from robosuite.utils.control_utils import *
from robosuite.controllers.ik import  InverseKinematicsController
from robosuite.controllers.ik import PyBulletServer
from robosuite.models import assets_root

import pybullet as p
import os
from os.path import join as pjoin

if __name__ == "__main__":

    pybullet_server = PyBulletServer()
    class PyBulletServer(object):
        """
        Helper class to encapsulate an alias for a single pybullet server
        """

        def __init__(self):
            # Attributes
            self.server_id = None
            self.is_active = False

            # Bodies: Dict of <bullet_robot_id : robot_name> active in pybullet simulation
            self.bodies = {}

            # Automatically setup this pybullet server
            self.connect()

        def connect(self):
            """
            Global function to (re-)connect to pybullet server instance if it's not currently active
            """
            if not self.is_active:
                self.server_id = p.connect(p.DIRECT)

                # Reset simulation (Assumes pre-existing connection to the PyBullet simulator)
                p.resetSimulation(physicsClientId=self.server_id)
                self.is_active = True

        def disconnect(self):
            """
            Function to disconnect and shut down this pybullet server instance.
            Should be called externally before resetting / instantiating a new controller
            """
            if self.is_active:
                p.disconnect(physicsClientId=self.server_id)
                self.bodies = {}
                self.is_active = False

    class InverseKinematicsController(JointVelocityController):
        """
        Controller for controlling robot arm via inverse kinematics. Allows position and orientation control of the
        robot's end effector.
        Inverse kinematics solving is handled by pybullet.
        NOTE: Control input actions are assumed to be relative to the current position / orientation of the end effector
        and are taken as the array (x_dpos, y_dpos, z_dpos, x_rot, y_rot, z_rot).
        Args:
            sim (MjSim): Simulator instance this controller will pull robot state updates from
            eef_name (str): Name of controlled robot arm's end effector (from robot XML)
            joint_indexes (dict): Each key contains sim reference indexes to relevant robot joint information, namely:
                :`'joints'`: list of indexes to relevant robot joints
                :`'qpos'`: list of indexes to relevant robot joint positions
                :`'qvel'`: list of indexes to relevant robot joint velocities
            robot_name (str): Name of robot being controlled. Can be {"Sawyer", "Panda", or "Baxter"}
            actuator_range (2-tuple of array of float): 2-Tuple (low, high) representing the robot joint actuator range
            eef_rot_offset (4-array): Quaternion (x,y,z,w) representing rotational offset between the final
                robot arm link coordinate system and the end effector coordinate system (i.e: the gripper)
            policy_freq (int): Frequency at which actions from the robot policy are fed into this controller
            ik_pos_limit (float): Limit (meters) above which the magnitude of a given action's
                positional inputs will be clipped
            ik_ori_limit (float): Limit (radians) above which the magnitude of a given action's
                orientation inputs will be clipped
            interpolator (Interpolator): Interpolator object to be used for interpolating from the current state to
                the goal state during each timestep between inputted actions
            converge_steps (int): How many iterations to run the pybullet inverse kinematics solver to converge to a
                solution
            **kwargs: Does nothing; placeholder to "sink" any additional arguments so that instantiating this controller
                via an argument dict that has additional extraneous arguments won't raise an error
        Raises:
            AssertionError: [Unsupported robot]
        """

        def __init__(
                self,
                sim,
                eef_name,
                joint_indexes,
                robot_name,
                actuator_range,
                eef_rot_offset,
                bullet_server_id=0,
                policy_freq=20,
                load_urdf=True,
                ik_pos_limit=None,
                ik_ori_limit=None,
                interpolator_pos=None,
                interpolator_ori=None,
                converge_steps=5,
                **kwargs,
        ):
            # Run sueprclass inits
            super().__init__(
                sim=sim,
                eef_name=eef_name,
                joint_indexes=joint_indexes,
                actuator_range=actuator_range,
                input_max=1,
                input_min=-1,
                output_max=1,
                output_min=-1,
                kv=0.25,
                policy_freq=policy_freq,
                velocity_limits=[-1, 1],
                **kwargs,
            )

            # Verify robot is supported by IK
            assert robot_name in SUPPORTED_IK_ROBOTS, (
                "Error: Tried to instantiate IK controller for unsupported robot! "
                "Inputted robot: {}, Supported robots: {}".format(robot_name, SUPPORTED_IK_ROBOTS)
            )

            # Initialize ik-specific attributes
            self.robot_name = Baxter  # Name of robot (e.g.: "Panda", "Sawyer", etc.)

            # Override underlying control dim
            self.control_dim = 6

            # Rotation offsets (for mujoco eef -> pybullet eef) and rest poses
            self.eef_rot_offset = eef_rot_offset
            self.rotation_offset = None
            self.rest_poses = None

            # Set the reference robot target pos / orientation (to prevent drift / weird ik numerical behavior over time)
            self.reference_target_pos = self.ee_pos
            self.reference_target_orn = T.mat2quat(self.ee_ori_mat)

            # Bullet server id
            self.bullet_server_id = bullet_server_id

            # Interpolator
            self.interpolator_pos = interpolator_pos
            self.interpolator_ori = interpolator_ori

            # Interpolator-related attributes
            self.ori_ref = None
            self.relative_ori = None

            # Values for initializing pybullet env
            self.ik_robot = None
            self.robot_urdf = None
            self.num_bullet_joints = None
            self.bullet_ee_idx = None
            self.bullet_joint_indexes = None  # Useful for splitting right and left hand indexes when controlling bimanual
            self.ik_command_indexes = None  # Relevant indices from ik loop; useful for splitting bimanual left / right
            self.ik_robot_target_pos_offset = None
            self.base_orn_offset_inv = None  # inverse orientation offset from pybullet base to world
            self.converge_steps = converge_steps

            # Set ik limits and override internal min / max
            self.ik_pos_limit = ik_pos_limit
            self.ik_ori_limit = ik_ori_limit

            # Target pos and ori
            self.ik_robot_target_pos = None
            self.ik_robot_target_orn = None  # note: this currently isn't being used at all

            # Commanded pos and resulting commanded vel
            self.commanded_joint_positions = None
            self.commanded_joint_velocities = None

            # Should be in (0, 1], smaller values mean less sensitivity.
            self.user_sensitivity = 0.3

            # Setup inverse kinematics
            self.setup_inverse_kinematics(load_urdf)

            # Lastly, sync pybullet state to mujoco state
            self.sync_state()

        def inverse_kinematics(self, target_position, target_orientation):

            ik_solution = list(
                p.calculateInverseKinematics(
                    bodyUniqueId=self.ik_robot,
                    endEffectorLinkIndex=self.bullet_ee_idx,
                    targetPosition=target_position,
                    targetOrientation=target_orientation,
                    lowerLimits=list(self.sim.model.jnt_range[self.joint_index, 0]),
                    upperLimits=list(self.sim.model.jnt_range[self.joint_index, 1]),
                    jointRanges=list(
                        self.sim.model.jnt_range[self.joint_index, 1] - self.sim.model.jnt_range[self.joint_index, 0]
                    ),
                    restPoses=self.rest_poses,
                    jointDamping=[0.1] * self.num_bullet_joints,
                    physicsClientId=self.bullet_server_id,
                )
            )
            return list(np.array(ik_solution)[self.ik_command_indexes])

        def setup_inverse_kinematics(self, load_urdf=True):
            # get paths to urdfs
            self.robot_urdf = pjoin(
                os.path.join(robosuite.models.assets_root, "bullet_data"),
                "{}_description/urdf/{}_arm.urdf".format(self.robot_name.lower(), self.robot_name.lower()),
            )

            # import reference to the global pybullet server and load the urdfs
            from robosuite.controllers import get_pybullet_server

            if load_urdf:
                self.ik_robot = p.loadURDF(fileName=self.robot_urdf, useFixedBase=1, physicsClientId=self.bullet_server_id)
                # Add this to the pybullet server
                get_pybullet_server().bodies[self.ik_robot] = self.robot_name
            else:
                self.ik_robot = max(get_pybullet_server().bodies)



            # load the number of joints from the bullet data
            self.num_bullet_joints = p.getNumJoints(self.ik_robot, physicsClientId=self.bullet_server_id)

            # Disable collisions between all the joints
            for joint in range(self.num_bullet_joints):
                p.setCollisionFilterGroupMask(
                    bodyUniqueId=self.ik_robot,
                    linkIndexA=joint,
                    collisionFilterGroup=0,
                    collisionFilterMask=0,
                    physicsClientId=self.bullet_server_id,
                )

                print(self.setup_inverse_kinematics(load_urdf=True))
                print(self.inverse_kinematics(5, 4, 5, 5, 3, 4))

