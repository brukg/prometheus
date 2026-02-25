#!/usr/bin/env python3
"""
Pink-IK ROS2 node for SO-100 5-DOF arm inverse kinematics control.

Subscribes to target poses, computes IK using Pink (Pinocchio-based),
and sends joint trajectories to the existing joint_trajectory_controller.

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
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration


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
        self.declare_parameter(
            "arm_joint_names",
            ["Shoulder_Rotation", "Shoulder_Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"],
        )
        self.declare_parameter("home_positions", [0.0, -1.7183, 1.5447, -0.3992, 0.0])

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

        # State
        self._joint_state_lock = threading.Lock()
        self._current_joint_positions = {}  # name -> angle (radians)
        self._model = None
        self._data = None
        self._configuration = None
        self._ee_task = None
        self._posture_task = None
        self._arm_joint_pin_ids = []   # pinocchio joint ids
        self._arm_joint_q_indices = [] # idx_q for each arm joint (cos/sin pair starts here)
        self._arm_joint_nqs = []       # nq for each arm joint (2 for continuous)
        self._solving = False

        # Build Pinocchio model from robot_description topic
        self._build_model()

        # Callback group for concurrent callbacks
        cb_group = ReentrantCallbackGroup()

        # Subscribers
        self._joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_state_cb,
            10,
            callback_group=cb_group,
        )

        self._target_sub = self.create_subscription(
            PoseStamped,
            "pink_ik/target_pose",
            self._target_pose_cb,
            10,
            callback_group=cb_group,
        )

        # Publishers
        self._ee_pose_pub = self.create_publisher(PoseStamped, "pink_ik/current_ee_pose", 10)
        self._status_pub = self.create_publisher(String, "pink_ik/status", 10)

        # Action client to joint_trajectory_controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            controller_action,
            callback_group=cb_group,
        )

        # Check action server availability once at startup
        self.get_logger().info("Waiting for joint_trajectory_controller action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn(
                "joint_trajectory_controller action server not available at startup. "
                "Will retry when sending trajectories."
            )
        else:
            self.get_logger().info("Action server connected.")

        # Timer to publish current EE pose at 10 Hz
        self._fk_timer = self.create_timer(0.1, self._publish_ee_pose, callback_group=cb_group)

        self._publish_status("idle")
        self.get_logger().info(
            f"Pink IK node ready. EE frame: {self._ee_frame}, "
            f"arm joints: {self._arm_joint_names}"
        )

    # ------------------------------------------------------------------ #
    # Angle <-> q conversion helpers for continuous (cos/sin) joints
    # ------------------------------------------------------------------ #

    def _angle_to_q(self, q, joint_idx, angle):
        """Set the q values for a continuous joint from an angle (radians).
        Continuous joints use q = [cos(theta), sin(theta)]."""
        q_idx = self._arm_joint_q_indices[joint_idx]
        nq = self._arm_joint_nqs[joint_idx]
        if nq == 2:
            q[q_idx] = math.cos(angle)
            q[q_idx + 1] = math.sin(angle)
        else:
            q[q_idx] = angle

    def _q_to_angle(self, q, joint_idx):
        """Extract the angle (radians) from q for a continuous joint.
        Continuous joints use q = [cos(theta), sin(theta)]."""
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
        # robot_state_publisher publishes robot_description as transient local
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._urdf_received = False
        self._urdf_string = ""

        def _desc_cb(msg):
            self._urdf_string = msg.data
            self._urdf_received = True

        sub = self.create_subscription(
            String, "/robot_description", _desc_cb, qos
        )

        self.get_logger().info("Waiting for /robot_description topic...")
        while not self._urdf_received:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.destroy_subscription(sub)

        urdf_string = self._urdf_string
        if not urdf_string:
            raise RuntimeError("Received empty robot_description")

        # Build Pinocchio model (kinematics only, no geometry/meshes needed)
        self._model = pin.buildModelFromXML(urdf_string)
        self._data = self._model.createData()

        # Find the URDF root link name for frame_id in published poses.
        # Pinocchio frame[0] is "universe", frame[1] is the actual URDF root link.
        self._urdf_root_frame = "base_footprint"
        if self._model.nframes > 1:
            self._urdf_root_frame = self._model.frames[1].name

        self.get_logger().info(
            f"Pinocchio model loaded: nq={self._model.nq}, nv={self._model.nv}, "
            f"{self._model.njoints} joints, {self._model.nframes} frames, "
            f"root link: {self._urdf_root_frame}"
        )

        # Map arm joint names to pinocchio model indices
        self._arm_joint_pin_ids = []
        self._arm_joint_q_indices = []
        self._arm_joint_nqs = []
        for idx, name in enumerate(self._arm_joint_names):
            if not self._model.existJointName(name):
                self.get_logger().error(f"Joint '{name}' not found in Pinocchio model!")
                raise RuntimeError(f"Joint '{name}' not found")
            jid = self._model.getJointId(name)
            joint = self._model.joints[jid]
            self._arm_joint_pin_ids.append(jid)
            self._arm_joint_q_indices.append(joint.idx_q)
            self._arm_joint_nqs.append(joint.nq)

            self.get_logger().info(
                f"  Joint '{name}': pin_id={jid}, q_idx={joint.idx_q}, nq={joint.nq}"
            )

        # Verify EE frame exists
        if not self._model.existFrame(self._ee_frame):
            self.get_logger().error(f"Frame '{self._ee_frame}' not found in model!")
            available = [self._model.frames[i].name for i in range(self._model.nframes)]
            self.get_logger().error(f"Available frames: {available}")
            raise RuntimeError(f"Frame '{self._ee_frame}' not found")

        ee_fid = self._model.getFrameId(self._ee_frame)
        self.get_logger().info(f"  EE frame '{self._ee_frame}': frame_id={ee_fid}")

        # Initialize configuration at home position
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

        # Set posture target to home
        q_home = pin.neutral(self._model)
        for i in range(len(self._arm_joint_names)):
            self._angle_to_q(q_home, i, self._home_positions[i])
        self._posture_task.set_target(q_home)

        self.get_logger().info("Pink IK tasks initialized.")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _joint_state_cb(self, msg: JointState):
        """Update internal joint state from joint_states topic."""
        with self._joint_state_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._current_joint_positions[name] = msg.position[i]

    def _target_pose_cb(self, msg: PoseStamped):
        """Handle incoming target pose - solve IK and execute trajectory."""
        if self._solving:
            self.get_logger().warn("Already solving IK, skipping new target.")
            return

        if not self._has_arm_state():
            self.get_logger().warn("No arm joint states received yet, cannot solve IK.")
            return

        self._solving = True
        self._publish_status("solving")

        try:
            target_se3 = self._pose_to_se3(msg.pose)
            q_current = self._get_current_q()

            self.get_logger().info(
                f"Target: pos=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
            )
            self.get_logger().info(f"Current q (arm angles): {self._get_current_arm_angles()}")

            self._configuration = Configuration(self._model, self._data, q_current)

            waypoints = self._solve_ik(target_se3)

            if waypoints is not None:
                self._publish_status("executing")
                self._send_trajectory(waypoints)
            else:
                self._publish_status("failed")
                self._solving = False

        except Exception as e:
            self.get_logger().error(f"IK solving failed: {e}")
            self.get_logger().error(traceback.format_exc())
            self._publish_status("failed")
            self._solving = False

    # ------------------------------------------------------------------ #
    # IK Solving
    # ------------------------------------------------------------------ #

    def _solve_ik(self, target_se3):
        """
        Run Pink differential IK loop.
        Returns list of arm joint angle waypoints, or None on failure.
        """
        self._ee_task.set_target(target_se3)

        tasks = [self._ee_task, self._posture_task]
        waypoints = []
        sample_interval = max(1, self._max_iters // self._num_waypoints)
        error = np.zeros(6)

        for i in range(self._max_iters):
            velocity = pink.solve_ik(
                self._configuration,
                tasks,
                self._dt,
                solver=self._solver,
                damping=self._lm_damping,
            )
            self._configuration.integrate_inplace(velocity, self._dt)

            # Sample waypoints at regular intervals
            if i % sample_interval == 0:
                arm_angles = self._extract_arm_angles(self._configuration.q)
                waypoints.append(arm_angles)

            # Check convergence
            ee_pose = self._configuration.get_transform_frame_to_world(self._ee_frame)
            error = pin.log6(ee_pose.actInv(target_se3)).vector
            pos_error = np.linalg.norm(error[:3])

            if pos_error < self._pos_threshold:
                self.get_logger().info(
                    f"IK converged in {i+1} iterations (pos_error={pos_error:.4f}m)"
                )
                arm_angles = self._extract_arm_angles(self._configuration.q)
                if not waypoints or waypoints[-1] != arm_angles:
                    waypoints.append(arm_angles)
                self.get_logger().info(f"Solution angles: {[f'{a:.3f}' for a in arm_angles]}")
                return waypoints

        pos_error_final = np.linalg.norm(error[:3])
        self.get_logger().warn(
            f"IK did not converge after {self._max_iters} iterations "
            f"(pos_error={pos_error_final:.4f}m). Sending best result."
        )
        arm_angles = self._extract_arm_angles(self._configuration.q)
        waypoints.append(arm_angles)
        return waypoints

    def _extract_arm_angles(self, q):
        """Extract arm joint angles from a full Pinocchio q vector."""
        return [self._q_to_angle(q, i) for i in range(len(self._arm_joint_names))]

    # ------------------------------------------------------------------ #
    # Trajectory execution
    # ------------------------------------------------------------------ #

    def _send_trajectory(self, waypoints):
        """Build and send a JointTrajectory to the controller action server."""
        if not self._action_client.server_is_ready():
            self.get_logger().error("joint_trajectory_controller action server not available!")
            self._publish_status("failed")
            self._solving = False
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._arm_joint_names)

        n = len(waypoints)
        for i, positions in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = positions
            t = self._traj_duration * (i + 1) / n
            secs = int(t)
            nsecs = int((t - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)
            trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"Sending trajectory with {n} waypoints over {self._traj_duration}s"
        )

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Trajectory goal rejected by controller.")
            self._publish_status("failed")
            self._solving = False
            return
        self.get_logger().info("Trajectory goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Trajectory execution succeeded.")
            self._publish_status("reached")
        else:
            self.get_logger().warn(f"Trajectory execution ended with status: {result.status}")
            self._publish_status("failed")
        self._solving = False

    # ------------------------------------------------------------------ #
    # FK / EE pose publishing
    # ------------------------------------------------------------------ #

    def _publish_ee_pose(self):
        """Compute FK and publish current end-effector pose."""
        if not self._has_arm_state():
            return

        q = self._get_current_q()
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)

        ee_fid = self._model.getFrameId(self._ee_frame)
        ee_se3 = self._data.oMf[ee_fid]

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

        self._ee_pose_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _has_arm_state(self) -> bool:
        """Check if we have joint state for all arm joints."""
        with self._joint_state_lock:
            return all(name in self._current_joint_positions for name in self._arm_joint_names)

    def _get_current_q(self) -> np.ndarray:
        """Build full Pinocchio q vector from current joint states (angles)."""
        q = pin.neutral(self._model)
        with self._joint_state_lock:
            for i, name in enumerate(self._arm_joint_names):
                if name in self._current_joint_positions:
                    angle = self._current_joint_positions[name]
                    self._angle_to_q(q, i, angle)
        return q

    def _get_current_arm_angles(self):
        """Get current arm angles from joint state for logging."""
        with self._joint_state_lock:
            return [
                f"{self._current_joint_positions.get(n, 0.0):.3f}"
                for n in self._arm_joint_names
            ]

    def _pose_to_se3(self, pose) -> pin.SE3:
        """Convert geometry_msgs/Pose to pinocchio SE3."""
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        quat = pin.Quaternion(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        )
        return pin.SE3(quat.matrix(), translation)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PinkIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
