#!/usr/bin/env python3
"""
Pink-IK ROS2 node for SO-100 5-DOF arm control.

Provides a unified ROS2 interface for arm manipulation:
  - MoveArm action server (cartesian, relative, named, home, joints modes)
  - GetArmState service (EE pose, joint positions, status, gripper)
  - SetGripper service (open/close/position control)

Uses Pink (Pinocchio-based differential IK) for Cartesian goals and sends
joint trajectories to the existing joint_trajectory_controller.

NOTE: The arm URDF uses 'continuous' joints. Pinocchio represents these
with (cos θ, sin θ) in q (nq=2 per joint), but velocity v uses a single
scalar (nv=1). All angle <-> q conversions must account for this.
"""

import math
import traceback
import threading
import numpy as np
import pinocchio as pin
import pink
from pink import Configuration
from pink.tasks import FrameTask, PostureTask

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration

from prometheus.action import MoveArm


class PinkIKNode(Node):
    def __init__(self):
        super().__init__("pink_ik_node")

        # Declare parameters
        self.declare_parameter("ee_frame", "Fixed_Gripper")
        self.declare_parameter("position_cost", 1.0)
        self.declare_parameter("orientation_cost", 0.1)
        self.declare_parameter("posture_cost", 1e-4)
        self.declare_parameter("lm_damping", 1e-3)
        self.declare_parameter("solver", "quadprog")
        self.declare_parameter("max_iterations", 300)
        self.declare_parameter("dt", 0.01)
        self.declare_parameter("position_threshold", 0.005)
        self.declare_parameter("trajectory_duration", 3.0)
        self.declare_parameter("num_waypoints", 20)
        self.declare_parameter("joint_states_topic", "prometheus/joint_states")
        self.declare_parameter("controller_action", "joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action", "gripper_controller/gripper_cmd")
        self.declare_parameter(
            "arm_joint_names",
            ["Shoulder_Rotation", "Shoulder_Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"],
        )
        self.declare_parameter("home_positions", [0.0, -1.7183, 1.5447, -0.3992, 0.0])

        # Named poses: declare each as a separate parameter list
        self.declare_parameter("named_poses.home", [0.0, -1.7183, 1.5447, -0.3992, 0.0])
        self.declare_parameter("named_poses.ready", [0.0, -0.5, 0.5, 0.0, 0.0])
        self.declare_parameter("named_poses.tucked", [0.0, -2.0, 2.0, -0.5, 0.0])

        # Read parameters
        self._ee_frame = self.get_parameter("ee_frame").value
        self._position_cost = self.get_parameter("position_cost").value
        self._orientation_cost = self.get_parameter("orientation_cost").value
        self._posture_cost = self.get_parameter("posture_cost").value
        self._lm_damping = self.get_parameter("lm_damping").value
        self._solver = self.get_parameter("solver").value
        self._max_iters = self.get_parameter("max_iterations").value
        self._dt = self.get_parameter("dt").value
        self._pos_threshold = self.get_parameter("position_threshold").value
        self._traj_duration = self.get_parameter("trajectory_duration").value
        self._num_waypoints = self.get_parameter("num_waypoints").value
        self._arm_joint_names = list(self.get_parameter("arm_joint_names").value)
        self._home_positions = list(self.get_parameter("home_positions").value)
        joint_states_topic = self.get_parameter("joint_states_topic").value
        controller_action = self.get_parameter("controller_action").value
        gripper_action = self.get_parameter("gripper_action").value

        # Build named poses dict
        self._named_poses = {}
        for name in ["home", "ready", "tucked"]:
            val = self.get_parameter(f"named_poses.{name}").value
            if val is not None:
                self._named_poses[name] = list(val)

        # State
        self._joint_state_lock = threading.Lock()
        self._current_joint_positions = {}  # name -> angle (radians)
        self._model = None
        self._data = None
        self._configuration = None
        self._ee_task = None
        self._posture_task = None
        self._arm_joint_pin_ids = []   # pinocchio joint ids
        self._arm_joint_q_indices = [] # idx_q for each arm joint
        self._arm_joint_nqs = []       # nq for each arm joint (2 for continuous)
        self._status = "idle"
        self._active_goal = None       # track active MoveArm goal

        # Build Pinocchio model from robot_description topic
        self._build_model()

        # Callback group for concurrent callbacks
        cb_group = ReentrantCallbackGroup()

        # Subscriber: joint states
        self._joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_state_cb,
            10,
            callback_group=cb_group,
        )

        # Publishers (kept for monitoring/visualization)
        self._ee_pose_pub = self.create_publisher(PoseStamped, "pink_ik/current_ee_pose", 10)
        self._status_pub = self.create_publisher(String, "pink_ik/status", 10)

        # Action client: joint_trajectory_controller
        self._jtc_client = ActionClient(
            self, FollowJointTrajectory, controller_action, callback_group=cb_group,
        )

        # Action client: gripper_controller
        self._gripper_client = ActionClient(
            self, GripperCommand, gripper_action, callback_group=cb_group,
        )

        # Action server: MoveArm
        self._move_arm_server = ActionServer(
            self,
            MoveArm,
            "arm/move_to_pose",
            execute_callback=self._execute_move_arm,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        # Wait for action servers
        self.get_logger().info("Waiting for controller action servers...")
        if not self._jtc_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("joint_trajectory_controller not available at startup.")
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("gripper_controller not available at startup.")

        # Timer: publish EE pose at 10 Hz
        self._fk_timer = self.create_timer(0.1, self._publish_ee_pose, callback_group=cb_group)

        self._set_status("idle")
        self.get_logger().info(
            f"Pink IK node ready. EE frame: {self._ee_frame}, "
            f"arm joints: {self._arm_joint_names}, "
            f"named poses: {list(self._named_poses.keys())}"
        )

    # ------------------------------------------------------------------ #
    # Angle <-> q conversion helpers for continuous (cos/sin) joints
    # ------------------------------------------------------------------ #

    def _angle_to_q(self, q, joint_idx, angle):
        """Set q values for a continuous joint from an angle (radians)."""
        q_idx = self._arm_joint_q_indices[joint_idx]
        nq = self._arm_joint_nqs[joint_idx]
        if nq == 2:
            q[q_idx] = math.cos(angle)
            q[q_idx + 1] = math.sin(angle)
        else:
            q[q_idx] = angle

    def _q_to_angle(self, q, joint_idx):
        """Extract angle (radians) from q for a continuous joint."""
        q_idx = self._arm_joint_q_indices[joint_idx]
        nq = self._arm_joint_nqs[joint_idx]
        if nq == 2:
            return math.atan2(q[q_idx + 1], q[q_idx])
        else:
            return float(q[q_idx])

    # ------------------------------------------------------------------ #
    # Model building
    # ------------------------------------------------------------------ #

    def _build_model(self):
        """Build Pinocchio model from /robot_description topic."""
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._urdf_received = False
        self._urdf_string = ""

        def _desc_cb(msg):
            self._urdf_string = msg.data
            self._urdf_received = True

        sub = self.create_subscription(String, "/robot_description", _desc_cb, qos)

        self.get_logger().info("Waiting for /robot_description topic...")
        while not self._urdf_received:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.destroy_subscription(sub)

        if not self._urdf_string:
            raise RuntimeError("Received empty robot_description")

        self._model = pin.buildModelFromXML(self._urdf_string)
        self._data = self._model.createData()

        # URDF root link (frame[0] is Pinocchio "universe", frame[1] is actual root)
        self._urdf_root_frame = "base_footprint"
        if self._model.nframes > 1:
            self._urdf_root_frame = self._model.frames[1].name

        self.get_logger().info(
            f"Pinocchio model: nq={self._model.nq}, nv={self._model.nv}, "
            f"{self._model.njoints} joints, root: {self._urdf_root_frame}"
        )

        # Map arm joints to pinocchio indices
        self._arm_joint_pin_ids = []
        self._arm_joint_q_indices = []
        self._arm_joint_nqs = []
        for name in self._arm_joint_names:
            if not self._model.existJointName(name):
                raise RuntimeError(f"Joint '{name}' not found in Pinocchio model")
            jid = self._model.getJointId(name)
            joint = self._model.joints[jid]
            self._arm_joint_pin_ids.append(jid)
            self._arm_joint_q_indices.append(joint.idx_q)
            self._arm_joint_nqs.append(joint.nq)
            self.get_logger().info(f"  Joint '{name}': pin_id={jid}, q_idx={joint.idx_q}, nq={joint.nq}")

        # Verify EE frame
        if not self._model.existFrame(self._ee_frame):
            available = [self._model.frames[i].name for i in range(self._model.nframes)]
            raise RuntimeError(f"Frame '{self._ee_frame}' not found. Available: {available}")

        # Initialize configuration at home
        q_init = pin.neutral(self._model)
        for i in range(len(self._arm_joint_names)):
            self._angle_to_q(q_init, i, self._home_positions[i])

        self._configuration = Configuration(self._model, self._data, q_init)

        # Create tasks
        self._ee_task = FrameTask(
            self._ee_frame,
            position_cost=self._position_cost,
            orientation_cost=self._orientation_cost,
        )
        self._posture_task = PostureTask(cost=self._posture_cost)

        q_home = pin.neutral(self._model)
        for i in range(len(self._arm_joint_names)):
            self._angle_to_q(q_home, i, self._home_positions[i])
        self._posture_task.set_target(q_home)

        self.get_logger().info("Pink IK tasks initialized.")

    # ------------------------------------------------------------------ #
    # MoveArm Action Server
    # ------------------------------------------------------------------ #

    def _goal_callback(self, goal_request):
        valid_modes = ("cartesian", "relative", "named", "home", "joints")
        if goal_request.mode not in valid_modes:
            self.get_logger().warn(f"Invalid mode '{goal_request.mode}', rejecting.")
            return GoalResponse.REJECT
        if self._active_goal is not None:
            self.get_logger().warn("Another goal is active, rejecting.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("MoveArm cancel requested.")
        return CancelResponse.ACCEPT

    async def _execute_move_arm(self, goal_handle):
        """Execute MoveArm action — main entry point for all arm movements."""
        self._active_goal = goal_handle
        request = goal_handle.request
        mode = request.mode
        result = MoveArm.Result()

        self.get_logger().info(f"MoveArm goal received: mode={mode}")

        try:
            if not self._has_arm_state():
                result.success = False
                result.message = "No arm joint states received yet"
                goal_handle.abort()
                self._active_goal = None
                return result

            duration = request.duration if request.duration > 0 else self._traj_duration

            if mode == "home":
                joint_targets = list(self._home_positions)
                success, msg = await self._execute_joint_trajectory(
                    joint_targets, duration, goal_handle
                )

            elif mode == "named":
                pose_name = request.pose_name
                if pose_name not in self._named_poses:
                    result.success = False
                    result.message = f"Unknown pose '{pose_name}'. Available: {list(self._named_poses.keys())}"
                    goal_handle.abort()
                    self._active_goal = None
                    return result
                joint_targets = self._named_poses[pose_name]
                success, msg = await self._execute_joint_trajectory(
                    joint_targets, duration, goal_handle
                )

            elif mode == "joints":
                joint_targets = list(request.joint_positions)
                if len(joint_targets) != len(self._arm_joint_names):
                    result.success = False
                    result.message = (
                        f"Expected {len(self._arm_joint_names)} joint positions, "
                        f"got {len(joint_targets)}"
                    )
                    goal_handle.abort()
                    self._active_goal = None
                    return result
                success, msg = await self._execute_joint_trajectory(
                    joint_targets, duration, goal_handle
                )

            elif mode == "cartesian":
                target_se3 = self._pose_to_se3(request.target_pose.pose)
                success, msg = await self._execute_ik_goal(
                    target_se3, duration, goal_handle
                )

            elif mode == "relative":
                # Get current EE pose, add offset
                current_se3 = self._get_current_ee_se3()
                offset = np.array([
                    request.offset.x, request.offset.y, request.offset.z
                ])
                target_translation = current_se3.translation + offset
                target_se3 = pin.SE3(current_se3.rotation, target_translation)
                self.get_logger().info(
                    f"Relative move: offset=({offset[0]:.3f}, {offset[1]:.3f}, {offset[2]:.3f}), "
                    f"target=({target_translation[0]:.3f}, {target_translation[1]:.3f}, {target_translation[2]:.3f})"
                )
                success, msg = await self._execute_ik_goal(
                    target_se3, duration, goal_handle
                )

            else:
                result.success = False
                result.message = f"Unhandled mode: {mode}"
                goal_handle.abort()
                self._active_goal = None
                return result

            # Build result
            result.success = success
            result.message = msg
            result.final_pose = self._get_current_ee_pose_msg()
            result.final_joint_positions = self._get_current_arm_angles_list()

            if success:
                goal_handle.succeed()
                self._set_status("reached")
            else:
                goal_handle.abort()
                self._set_status("failed")

        except Exception as e:
            self.get_logger().error(f"MoveArm failed: {e}")
            self.get_logger().error(traceback.format_exc())
            result.success = False
            result.message = str(e)
            goal_handle.abort()
            self._set_status("failed")

        self._active_goal = None
        return result

    async def _execute_ik_goal(self, target_se3, duration, goal_handle):
        """Solve IK for a Cartesian target and execute trajectory."""
        self._set_status("solving")

        q_current = self._get_current_q()
        self._configuration = Configuration(self._model, self._data, q_current)
        self._ee_task.set_target(target_se3)

        tasks = [self._ee_task, self._posture_task]
        waypoints = []
        sample_interval = max(1, self._max_iters // self._num_waypoints)
        error = np.zeros(6)

        for i in range(self._max_iters):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self._set_status("idle")
                goal_handle.canceled()
                return False, "Cancelled during IK solving"

            velocity = pink.solve_ik(
                self._configuration, tasks, self._dt,
                solver=self._solver, damping=self._lm_damping,
            )
            self._configuration.integrate_inplace(velocity, self._dt)

            if i % sample_interval == 0:
                arm_angles = self._extract_arm_angles(self._configuration.q)
                waypoints.append(arm_angles)

            # Check convergence
            ee_pose = self._configuration.get_transform_frame_to_world(self._ee_frame)
            error = pin.log6(ee_pose.actInv(target_se3)).vector
            pos_error = np.linalg.norm(error[:3])

            # Publish feedback periodically
            if i % 50 == 0:
                feedback = MoveArm.Feedback()
                feedback.progress = min(1.0, i / self._max_iters)
                feedback.position_error = float(pos_error)
                feedback.phase = "solving"
                goal_handle.publish_feedback(feedback)

            if pos_error < self._pos_threshold:
                self.get_logger().info(f"IK converged in {i+1} iters (error={pos_error:.4f}m)")
                arm_angles = self._extract_arm_angles(self._configuration.q)
                if not waypoints or waypoints[-1] != arm_angles:
                    waypoints.append(arm_angles)
                break

        if np.linalg.norm(error[:3]) >= self._pos_threshold:
            pos_err = np.linalg.norm(error[:3])
            self.get_logger().warn(
                f"IK did not converge ({pos_err:.4f}m). Sending best result."
            )
            arm_angles = self._extract_arm_angles(self._configuration.q)
            waypoints.append(arm_angles)

        # Execute trajectory
        self._set_status("executing")
        return await self._send_trajectory_and_wait(waypoints, duration, goal_handle)

    async def _execute_joint_trajectory(self, joint_targets, duration, goal_handle):
        """Send a direct joint trajectory (for home/named/joints modes)."""
        self._set_status("executing")

        # Publish solving feedback
        feedback = MoveArm.Feedback()
        feedback.progress = 0.5
        feedback.position_error = 0.0
        feedback.phase = "executing"
        goal_handle.publish_feedback(feedback)

        return await self._send_trajectory_and_wait([joint_targets], duration, goal_handle)

    async def _send_trajectory_and_wait(self, waypoints, duration, goal_handle):
        """Build JointTrajectory, send to controller, wait for result."""
        if not self._jtc_client.server_is_ready():
            return False, "joint_trajectory_controller action server not available"

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._arm_joint_names)

        n = len(waypoints)
        for i, positions in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            t = duration * (i + 1) / n
            secs = int(t)
            nsecs = int((t - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)
            trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(f"Sending trajectory: {n} waypoints, {duration}s")

        send_future = self._jtc_client.send_goal_async(goal)
        goal_response = await send_future

        if not goal_response.accepted:
            return False, "Trajectory goal rejected by controller"

        self.get_logger().info("Trajectory accepted, waiting for execution...")

        # Publish executing feedback
        feedback = MoveArm.Feedback()
        feedback.progress = 0.75
        feedback.position_error = 0.0
        feedback.phase = "executing"
        goal_handle.publish_feedback(feedback)

        result_future = goal_response.get_result_async()
        result_response = await result_future

        if result_response.status == GoalStatus.STATUS_SUCCEEDED:
            # Final feedback
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            return True, "Trajectory execution succeeded"
        else:
            return False, f"Trajectory execution failed (status={result_response.status})"

    # ------------------------------------------------------------------ #
    # Joint state callback
    # ------------------------------------------------------------------ #

    def _joint_state_cb(self, msg: JointState):
        """Update internal joint state from joint_states topic."""
        with self._joint_state_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._current_joint_positions[name] = msg.position[i]

    # ------------------------------------------------------------------ #
    # FK / EE pose publishing
    # ------------------------------------------------------------------ #

    def _publish_ee_pose(self):
        """Compute FK and publish current end-effector pose."""
        if not self._has_arm_state():
            return
        msg = self._get_current_ee_pose_msg()
        self._ee_pose_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # IK helpers
    # ------------------------------------------------------------------ #

    def _solve_ik(self, target_se3):
        """Run Pink differential IK loop. Returns waypoints or None."""
        self._ee_task.set_target(target_se3)
        tasks = [self._ee_task, self._posture_task]
        waypoints = []
        sample_interval = max(1, self._max_iters // self._num_waypoints)
        error = np.zeros(6)

        for i in range(self._max_iters):
            velocity = pink.solve_ik(
                self._configuration, tasks, self._dt,
                solver=self._solver, damping=self._lm_damping,
            )
            self._configuration.integrate_inplace(velocity, self._dt)

            if i % sample_interval == 0:
                waypoints.append(self._extract_arm_angles(self._configuration.q))

            ee_pose = self._configuration.get_transform_frame_to_world(self._ee_frame)
            error = pin.log6(ee_pose.actInv(target_se3)).vector
            pos_error = np.linalg.norm(error[:3])

            if pos_error < self._pos_threshold:
                arm_angles = self._extract_arm_angles(self._configuration.q)
                if not waypoints or waypoints[-1] != arm_angles:
                    waypoints.append(arm_angles)
                return waypoints

        waypoints.append(self._extract_arm_angles(self._configuration.q))
        return waypoints

    def _extract_arm_angles(self, q):
        """Extract arm joint angles from a full Pinocchio q vector."""
        return [self._q_to_angle(q, i) for i in range(len(self._arm_joint_names))]

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _has_arm_state(self) -> bool:
        with self._joint_state_lock:
            return all(name in self._current_joint_positions for name in self._arm_joint_names)

    def _get_current_q(self) -> np.ndarray:
        """Build full Pinocchio q from current joint states."""
        q = pin.neutral(self._model)
        with self._joint_state_lock:
            for i, name in enumerate(self._arm_joint_names):
                if name in self._current_joint_positions:
                    self._angle_to_q(q, i, self._current_joint_positions[name])
        return q

    def _get_current_arm_angles_list(self):
        """Get current arm joint angles as a list of floats."""
        with self._joint_state_lock:
            return [
                float(self._current_joint_positions.get(n, 0.0))
                for n in self._arm_joint_names
            ]

    def _get_current_ee_se3(self):
        """Compute current EE SE3 pose via FK."""
        q = self._get_current_q()
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        ee_fid = self._model.getFrameId(self._ee_frame)
        return self._data.oMf[ee_fid]

    def _get_current_ee_pose_msg(self) -> PoseStamped:
        """Get current EE pose as a PoseStamped message."""
        ee_se3 = self._get_current_ee_se3()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._urdf_root_frame
        msg.pose.position.x = float(ee_se3.translation[0])
        msg.pose.position.y = float(ee_se3.translation[1])
        msg.pose.position.z = float(ee_se3.translation[2])
        quat = pin.Quaternion(ee_se3.rotation)
        msg.pose.orientation.x = float(quat.x)
        msg.pose.orientation.y = float(quat.y)
        msg.pose.orientation.z = float(quat.z)
        msg.pose.orientation.w = float(quat.w)
        return msg

    def _pose_to_se3(self, pose) -> pin.SE3:
        """Convert geometry_msgs/Pose to pinocchio SE3."""
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = pin.Quaternion(
            pose.orientation.w, pose.orientation.x,
            pose.orientation.y, pose.orientation.z,
        )
        return pin.SE3(quat.matrix(), translation)

    def _set_status(self, status: str):
        self._status = status
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _publish_status(self, status: str):
        self._set_status(status)


def main(args=None):
    rclpy.init(args=args)
    node = PinkIKNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
