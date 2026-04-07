import math
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


class PickPlaceDemoNode(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_demo")
        self.get_logger().info("Pick and Place最小Demo节点启动")

        self._declare_parameters()

        move_action_name = self.get_parameter("move_action_name").get_parameter_value().string_value
        gripper_action_name = self.get_parameter("gripper_action_name").get_parameter_value().string_value

        self.move_group_client = ActionClient(self, MoveGroup, move_action_name)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, gripper_action_name)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        preferred_set_state_srv = self.get_parameter("set_entity_state_service").get_parameter_value().string_value
        self._set_entity_state_clients = {
            preferred_set_state_srv: self.create_client(SetEntityState, preferred_set_state_srv),
            "/set_entity_state": self.create_client(SetEntityState, "/set_entity_state"),
            "/gazebo/set_entity_state": self.create_client(SetEntityState, "/gazebo/set_entity_state"),
        }
        self._active_set_state_service_name = preferred_set_state_srv
        self.delete_entity_client = self.create_client(DeleteEntity, "/delete_entity")
        self.spawn_entity_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.attach_link_client = self.create_client(AttachLink, self.get_parameter("attach_service").value)
        self.detach_link_client = self.create_client(DetachLink, self.get_parameter("detach_service").value)

        self._sequence: List[Tuple[str, object]] = []
        self._current_step = -1
        self._running = False
        self._active_goal_type: Optional[str] = None
        self._gripper_last_command: Optional[float] = None
        self._current_move_target: Optional[Tuple[float, float, float]] = None
        self._current_move_retry = 0
        self._object_attached = False
        self._set_state_in_flight = False
        self._fallback_replace_in_flight = False
        self._warned_no_set_state_service = False
        self._delete_entity_in_flight = False
        self._spawn_entity_in_flight = False
        self._object_hidden_in_transport = False
        self._attach_in_flight = False
        self._detach_in_flight = False
        self._physical_attached = False
        self._object_prepared = False
        self._object_prepare_in_flight = False

        model_path = self.get_parameter("object_model_sdf_path").get_parameter_value().string_value
        self._object_model_xml = self._load_object_model_xml(model_path)

        follow_rate = self.get_parameter("object_follow_rate_hz").get_parameter_value().double_value
        self._follow_timer = self.create_timer(1.0 / max(follow_rate, 1.0), self._follow_object_timer_cb)

        self._start_timer = self.create_timer(1.0, self._try_start_demo)

    def _declare_parameters(self) -> None:
        self.declare_parameter("move_action_name", "/move_action")
        self.declare_parameter("gripper_action_name", "/gripper_controller/follow_joint_trajectory")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ee_link", "gripper")
        self.declare_parameter("reference_frame", "base")
        self.declare_parameter("pipeline_id", "")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("max_velocity_scaling_factor", 0.25)
        self.declare_parameter("max_acceleration_scaling_factor", 0.25)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("orientation_tolerance_xyz", [0.05, 0.05, 0.05])
        self.declare_parameter("target_orientation", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("use_orientation_constraint", True)

        self.declare_parameter("pick_position", [0.23, 0.06, 0.09])
        self.declare_parameter("place_position", [0.22, -0.10, 0.09])
        self.declare_parameter("approach_offset_z", 0.06)
        self.declare_parameter("retreat_offset_z", 0.06)

        self.declare_parameter("gripper_joint_name", "6")
        self.declare_parameter("gripper_open", 0.0)
        self.declare_parameter("gripper_close", 0.8)
        self.declare_parameter("gripper_motion_time", 1.5)

        self.declare_parameter("object_name", "target_box")
        self.declare_parameter("object_link_name", "box_link")
        self.declare_parameter("robot_model_name", "so101")
        self.declare_parameter("robot_link_name", "gripper")
        self.declare_parameter("object_spawn_x", -0.16)
        self.declare_parameter("object_spawn_y", 0.02)
        self.declare_parameter("object_spawn_z", 0.14)
        self.declare_parameter("object_spawn_reference_frame", "world")
        self.declare_parameter("use_physical_attach", True)
        self.declare_parameter("attach_service", "/ATTACHLINK")
        self.declare_parameter("detach_service", "/DETACHLINK")
        self.declare_parameter("set_entity_state_service", "/gazebo/set_entity_state")
        self.declare_parameter("use_object_follow", True)
        self.declare_parameter("use_delete_spawn_fallback", True)
        self.declare_parameter("object_follow_rate_hz", 20.0)
        self.declare_parameter("attached_object_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "object_model_sdf_path",
            f"{get_package_share_directory('position_topic')}/models/target_box.sdf",
        )

        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("move_retry_max", 2)
        self.declare_parameter("move_retry_delay_sec", 0.5)
        self.declare_parameter("post_gripper_settle_sec", 0.25)

    def _try_start_demo(self) -> None:
        if self._running:
            return

        move_ready = self.move_group_client.server_is_ready()
        grip_ready = self.gripper_client.server_is_ready()

        if not move_ready or not grip_ready:
            self.get_logger().info(
                f"等待Action服务... move_group={move_ready}, gripper={grip_ready}",
                throttle_duration_sec=2.0,
            )
            return

        start_delay = self.get_parameter("start_delay_sec").get_parameter_value().double_value
        if self.get_clock().now().nanoseconds < int(start_delay * 1e9):
            return

        if not self._object_prepared:
            self._ensure_object_prepared()
            return

        self._sequence = self._build_sequence()
        self._current_step = -1
        self._running = True
        self._start_timer.cancel()

        self.get_logger().info("Action服务就绪，开始执行Pick and Place流程")
        self._run_next_step()

    def _build_sequence(self) -> List[Tuple[str, object]]:
        pick = self._get_position_param("pick_position")
        place = self._get_position_param("place_position")
        approach = self.get_parameter("approach_offset_z").get_parameter_value().double_value
        retreat = self.get_parameter("retreat_offset_z").get_parameter_value().double_value
        gripper_open = self.get_parameter("gripper_open").get_parameter_value().double_value
        gripper_close = self.get_parameter("gripper_close").get_parameter_value().double_value

        pre_grasp = (pick[0], pick[1], pick[2] + approach)
        lift_after_pick = (pick[0], pick[1], pick[2] + retreat)
        pre_place = (place[0], place[1], place[2] + approach)
        retreat_after_place = (place[0], place[1], place[2] + retreat)

        return [
            ("gripper", gripper_open),
            ("move", pre_grasp),
            ("move", pick),
            ("gripper", gripper_close),
            ("move", lift_after_pick),
            ("move", pre_place),
            ("move", place),
            ("gripper", gripper_open),
            ("move", retreat_after_place),
        ]

    def _get_position_param(self, name: str) -> Tuple[float, float, float]:
        raw = self.get_parameter(name).get_parameter_value().double_array_value
        if len(raw) != 3:
            raise ValueError(f"参数{name}长度必须是3")
        return float(raw[0]), float(raw[1]), float(raw[2])

    def _run_next_step(self) -> None:
        self._current_step += 1
        if self._current_step >= len(self._sequence):
            self._running = False
            self.get_logger().info("Pick and Place流程执行完成")
            return

        step_type, step_value = self._sequence[self._current_step]
        self.get_logger().info(f"执行步骤[{self._current_step + 1}/{len(self._sequence)}]: {step_type} -> {step_value}")

        if step_type == "move":
            target = step_value
            self._current_move_target = (float(target[0]), float(target[1]), float(target[2]))
            self._current_move_retry = 0
            self._send_move_goal(float(target[0]), float(target[1]), float(target[2]))
            return

        if step_type == "gripper":
            self._send_gripper_goal(float(step_value))
            return

        self.get_logger().error(f"未知步骤类型: {step_type}")
        self._running = False

    def _send_move_goal(self, x: float, y: float, z: float) -> None:
        goal = MoveGroup.Goal()
        goal.request = self._build_motion_plan_request(self._build_pose_stamped(x, y, z))
        goal.planning_options = self._build_planning_options()

        self._active_goal_type = "move"
        send_future = self.move_group_client.send_goal_async(goal)
        send_future.add_done_callback(self._move_goal_response_callback)

    def _build_pose_stamped(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("reference_frame").get_parameter_value().string_value

        orientation = self.get_parameter("target_orientation").get_parameter_value().double_array_value
        if len(orientation) != 4:
            self.get_logger().warn("target_orientation长度不是4，回退到单位四元数")
            orientation = [0.0, 0.0, 0.0, 1.0]

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = float(orientation[0])
        msg.pose.orientation.y = float(orientation[1])
        msg.pose.orientation.z = float(orientation[2])
        msg.pose.orientation.w = float(orientation[3])
        return msg

    def _build_motion_plan_request(self, target: PoseStamped) -> MotionPlanRequest:
        request = MotionPlanRequest()

        request.pipeline_id = self.get_parameter("pipeline_id").get_parameter_value().string_value
        request.planner_id = self.get_parameter("planner_id").get_parameter_value().string_value
        request.group_name = self.get_parameter("group_name").get_parameter_value().string_value
        request.num_planning_attempts = (
            self.get_parameter("num_planning_attempts").get_parameter_value().integer_value
        )
        request.allowed_planning_time = (
            self.get_parameter("allowed_planning_time").get_parameter_value().double_value
        )
        request.max_velocity_scaling_factor = (
            self.get_parameter("max_velocity_scaling_factor").get_parameter_value().double_value
        )
        request.max_acceleration_scaling_factor = (
            self.get_parameter("max_acceleration_scaling_factor").get_parameter_value().double_value
        )

        ee_link = self.get_parameter("ee_link").get_parameter_value().string_value
        frame_id = target.header.frame_id
        position_tolerance = self.get_parameter("position_tolerance").get_parameter_value().double_value
        orientation_tolerance = self.get_parameter("orientation_tolerance_xyz").get_parameter_value().double_array_value
        if len(orientation_tolerance) != 3:
            orientation_tolerance = [0.05, 0.05, 0.05]

        constraints = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = ee_link
        pos_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [max(position_tolerance, 1e-4)]

        region = BoundingVolume()
        region.primitives.append(sphere)

        target_pose = Pose()
        target_pose.position = target.pose.position
        target_pose.orientation.w = 1.0
        region.primitive_poses.append(target_pose)
        pos_constraint.constraint_region = region

        constraints.position_constraints.append(pos_constraint)
        use_ori = self.get_parameter("use_orientation_constraint").get_parameter_value().bool_value
        if use_ori:
            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = frame_id
            ori_constraint.link_name = ee_link
            ori_constraint.orientation = target.pose.orientation
            ori_constraint.absolute_x_axis_tolerance = float(orientation_tolerance[0])
            ori_constraint.absolute_y_axis_tolerance = float(orientation_tolerance[1])
            ori_constraint.absolute_z_axis_tolerance = float(orientation_tolerance[2])
            ori_constraint.weight = 1.0
            constraints.orientation_constraints.append(ori_constraint)
        request.goal_constraints.append(constraints)
        return request

    def _build_planning_options(self) -> PlanningOptions:
        options = PlanningOptions()
        options.plan_only = False
        options.look_around = False
        options.look_around_attempts = 0
        options.max_safe_execution_cost = 0.0
        options.replan = False
        options.replan_attempts = 0
        options.replan_delay = 0.0
        return options

    def _move_goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"发送move目标失败: {exc}")
            self._running = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup拒绝了目标")
            self._running = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._move_result_callback)

    def _move_result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"获取move结果失败: {exc}")
            self._running = False
            return

        status = wrapped_result.status
        result = wrapped_result.result
        if status != GoalStatus.STATUS_SUCCEEDED:
            max_retry = self.get_parameter("move_retry_max").get_parameter_value().integer_value
            retry_delay = self.get_parameter("move_retry_delay_sec").get_parameter_value().double_value
            if self._current_move_target is not None and self._current_move_retry < max_retry:
                self._current_move_retry += 1
                self.get_logger().warn(
                    "MoveGroup执行失败，准备重试 "
                    f"({self._current_move_retry}/{max_retry}): status={status}, "
                    f"error_code={result.error_code.val}"
                )

                x, y, z = self._current_move_target
                self._call_once_after(retry_delay, lambda: self._send_move_goal(x, y, z))
                return

            self.get_logger().error(
                f"MoveGroup执行失败: status={status}, error_code={result.error_code.val}, planning_time={result.planning_time:.3f}s"
            )
            self._running = False
            return

        self.get_logger().info(
            f"MoveGroup执行成功: error_code={result.error_code.val}, planning_time={result.planning_time:.3f}s"
        )
        self._current_move_target = None
        self._current_move_retry = 0
        self._active_goal_type = None
        self._run_next_step()

    def _send_gripper_goal(self, target_joint_value: float) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            self.get_parameter("gripper_joint_name").get_parameter_value().string_value
        ]

        motion_time = self.get_parameter("gripper_motion_time").get_parameter_value().double_value
        sec = int(math.floor(motion_time))
        nanosec = int((motion_time - sec) * 1e9)

        point = JointTrajectoryPoint()
        point.positions = [target_joint_value]
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal.trajectory.points = [point]

        self._active_goal_type = "gripper"
        self._gripper_last_command = target_joint_value
        send_future = self.gripper_client.send_goal_async(goal)
        send_future.add_done_callback(self._gripper_goal_response_callback)

    def _gripper_goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"发送gripper目标失败: {exc}")
            self._running = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("Gripper控制器拒绝了目标")
            self._running = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._gripper_result_callback)

    def _gripper_result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"获取gripper结果失败: {exc}")
            self._running = False
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"Gripper执行失败: status={wrapped_result.status}")
            self._running = False
            return

        self.get_logger().info("Gripper执行成功")
        self._update_object_attach_state_after_gripper_action()
        self._active_goal_type = None
        settle = self.get_parameter("post_gripper_settle_sec").get_parameter_value().double_value
        self._call_once_after(settle, self._run_next_step)

    def _update_object_attach_state_after_gripper_action(self) -> None:
        if self._gripper_last_command is None:
            return

        close_cmd = self.get_parameter("gripper_close").get_parameter_value().double_value
        open_cmd = self.get_parameter("gripper_open").get_parameter_value().double_value
        eps = 1e-3

        if abs(self._gripper_last_command - close_cmd) < eps:
            self._object_attached = True
            if self._try_attach_links():
                self.get_logger().info("已请求物理附着: 等待Gazebo附着结果")
            else:
                use_object_follow = self.get_parameter("use_object_follow").get_parameter_value().bool_value
                if use_object_follow:
                    self.get_logger().info("物理附着不可用，回退到可视跟随模式")
                    self._update_attached_object_pose()
                else:
                    self.get_logger().warn("物理附着不可用，且可视跟随已关闭")
            return

        if abs(self._gripper_last_command - open_cmd) < eps:
            self._object_attached = False
            if not self._physical_attached:
                self.get_logger().info("当前无物理附着，跳过解附着请求")
                return
            if self._try_detach_links():
                self.get_logger().info("已请求物理解附着: 等待Gazebo解附着结果")
            else:
                self.get_logger().info("已解除附着跟随模式: 目标物保持当前位置")

    def _try_attach_links(self) -> bool:
        use_physical_attach = self.get_parameter("use_physical_attach").get_parameter_value().bool_value
        if not use_physical_attach:
            return False
        if self._attach_in_flight:
            return True
        if not self.attach_link_client.service_is_ready():
            return False

        req = AttachLink.Request()
        req.model1_name = self.get_parameter("robot_model_name").get_parameter_value().string_value
        req.link1_name = self.get_parameter("robot_link_name").get_parameter_value().string_value
        req.model2_name = self.get_parameter("object_name").get_parameter_value().string_value
        req.link2_name = self.get_parameter("object_link_name").get_parameter_value().string_value

        self._attach_in_flight = True
        future = self.attach_link_client.call_async(req)
        future.add_done_callback(self._attach_done_callback)
        return True

    def _try_detach_links(self) -> bool:
        use_physical_attach = self.get_parameter("use_physical_attach").get_parameter_value().bool_value
        if not use_physical_attach:
            return False
        if self._detach_in_flight:
            return True
        if not self.detach_link_client.service_is_ready():
            return False

        req = DetachLink.Request()
        req.model1_name = self.get_parameter("robot_model_name").get_parameter_value().string_value
        req.link1_name = self.get_parameter("robot_link_name").get_parameter_value().string_value
        req.model2_name = self.get_parameter("object_name").get_parameter_value().string_value
        req.link2_name = self.get_parameter("object_link_name").get_parameter_value().string_value

        self._detach_in_flight = True
        future = self.detach_link_client.call_async(req)
        future.add_done_callback(self._detach_done_callback)
        return True

    def _attach_done_callback(self, future) -> None:
        self._attach_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"物理附着调用失败: {exc}")
            self._physical_attached = False
            return

        if not response.success:
            self.get_logger().warn(f"物理附着失败: {response.message}")
            self._physical_attached = False
            return

        self._physical_attached = True
        self.get_logger().info("物理附着成功: 目标物将由Gazebo关节约束跟随夹爪")

    def _detach_done_callback(self, future) -> None:
        self._detach_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"物理解附着调用失败: {exc}")
            return

        if not response.success:
            self.get_logger().warn(f"物理解附着失败: {response.message}")
            return

        self._physical_attached = False
        self.get_logger().info("物理解附着成功: 目标物已从夹爪释放")

    def _follow_object_timer_cb(self) -> None:
        if not self._object_attached:
            return
        if not self.get_parameter("use_object_follow").get_parameter_value().bool_value:
            return
        if self.get_parameter("use_physical_attach").get_parameter_value().bool_value and self.attach_link_client.service_is_ready():
            return
        self._update_attached_object_pose()

    def _ensure_object_prepared(self) -> bool:
        if not self._object_model_xml:
            self.get_logger().warn("目标模型为空，跳过目标初始化")
            return True
        if self._object_prepare_in_flight:
            return False
        if not self.spawn_entity_client.service_is_ready():
            self.get_logger().info("等待Gazebo对象服务就绪...", throttle_duration_sec=2.0)
            return False

        object_name = self.get_parameter("object_name").get_parameter_value().string_value
        reference_frame = self.get_parameter("object_spawn_reference_frame").get_parameter_value().string_value

        pose = Pose()
        pose.position.x = self.get_parameter("object_spawn_x").get_parameter_value().double_value
        pose.position.y = self.get_parameter("object_spawn_y").get_parameter_value().double_value
        pose.position.z = self.get_parameter("object_spawn_z").get_parameter_value().double_value
        pose.orientation.w = 1.0

        spawn_req = SpawnEntity.Request()
        spawn_req.name = object_name
        spawn_req.xml = self._object_model_xml
        spawn_req.robot_namespace = ""
        spawn_req.initial_pose = pose
        spawn_req.reference_frame = reference_frame

        self._object_prepare_in_flight = True
        future = self.spawn_entity_client.call_async(spawn_req)
        future.add_done_callback(
            lambda f, p=pose: self._object_prepare_spawn_done_callback(f, p)
        )
        return True

    def _object_prepare_spawn_done_callback(self, future, pose: Pose) -> None:
        self._object_prepare_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"目标物生成失败，将重试: {exc}")
            return

        status_message = (response.status_message or "").lower()
        if response.success or "already exists" in status_message:
            self._object_prepared = True
            self.get_logger().info(
                "目标物已就绪: "
                f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
            )
            return

        self.get_logger().warn(f"目标物生成失败，将重试: {response.status_message}")

    def _call_service_sync(self, client, request, timeout_sec: float):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception:  # noqa: BLE001
            return None

    def _update_attached_object_pose(self) -> None:
        if self._set_state_in_flight:
            return

        set_state_client = self._select_set_entity_state_client()
        if set_state_client is None:
            if not self._warned_no_set_state_service:
                self._warned_no_set_state_service = True
                self.get_logger().warn("实体状态服务不可用，切换到Gazebo Classic可视回退模式")
            self._update_attached_object_pose_with_replace()
            return

        self._warned_no_set_state_service = False

        reference_frame = self.get_parameter("reference_frame").get_parameter_value().string_value
        ee_link = self.get_parameter("ee_link").get_parameter_value().string_value
        offset = self.get_parameter("attached_object_offset_xyz").get_parameter_value().double_array_value
        if len(offset) != 3:
            offset = [0.0, 0.0, 0.0]

        try:
            transform = self.tf_buffer.lookup_transform(reference_frame, ee_link, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(f"读取TF失败，无法更新目标物位置: {exc}", throttle_duration_sec=2.0)
            return

        pose = Pose()
        pose.position.x = transform.transform.translation.x + float(offset[0])
        pose.position.y = transform.transform.translation.y + float(offset[1])
        pose.position.z = transform.transform.translation.z + float(offset[2])
        pose.orientation = transform.transform.rotation

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.get_parameter("object_name").get_parameter_value().string_value
        req.state.pose = pose
        req.state.reference_frame = reference_frame

        self._set_state_in_flight = True
        future = set_state_client.call_async(req)
        future.add_done_callback(self._set_entity_state_done_callback)

    def _update_attached_object_pose_with_replace(self) -> None:
        use_fallback = self.get_parameter("use_delete_spawn_fallback").get_parameter_value().bool_value
        if not use_fallback or self._fallback_replace_in_flight:
            return
        if not self._object_model_xml:
            return
        if not self.delete_entity_client.service_is_ready() or not self.spawn_entity_client.service_is_ready():
            return

        reference_frame = self.get_parameter("reference_frame").get_parameter_value().string_value
        ee_link = self.get_parameter("ee_link").get_parameter_value().string_value
        offset = self.get_parameter("attached_object_offset_xyz").get_parameter_value().double_array_value
        if len(offset) != 3:
            offset = [0.0, 0.0, 0.0]

        try:
            transform = self.tf_buffer.lookup_transform(reference_frame, ee_link, rclpy.time.Time())
        except TransformException:
            return

        pose = Pose()
        pose.position.x = transform.transform.translation.x + float(offset[0])
        pose.position.y = transform.transform.translation.y + float(offset[1])
        pose.position.z = transform.transform.translation.z + float(offset[2])
        pose.orientation = transform.transform.rotation

        self._fallback_replace_in_flight = True
        delete_req = DeleteEntity.Request()
        delete_req.name = self.get_parameter("object_name").get_parameter_value().string_value
        delete_future = self.delete_entity_client.call_async(delete_req)
        delete_future.add_done_callback(lambda f: self._fallback_delete_then_spawn_cb(f, pose, reference_frame))

    def _fallback_delete_then_spawn_cb(self, future, pose: Pose, reference_frame: str) -> None:
        # 删除失败也继续spawn，确保模型能在夹爪附近可见。
        try:
            future.result()
        except Exception:
            pass

        spawn_req = SpawnEntity.Request()
        spawn_req.name = self.get_parameter("object_name").get_parameter_value().string_value
        spawn_req.xml = self._object_model_xml
        spawn_req.robot_namespace = ""
        spawn_req.initial_pose = pose
        spawn_req.reference_frame = reference_frame

        spawn_future = self.spawn_entity_client.call_async(spawn_req)
        spawn_future.add_done_callback(self._fallback_spawn_done_cb)

    def _fallback_spawn_done_cb(self, future) -> None:
        self._fallback_replace_in_flight = False
        try:
            future.result()
        except Exception:
            pass

    def _select_set_entity_state_client(self):
        active = self._set_entity_state_clients.get(self._active_set_state_service_name)
        if active is not None and active.service_is_ready():
            return active

        for name, client in self._set_entity_state_clients.items():
            if client.service_is_ready():
                if name != self._active_set_state_service_name:
                    self._active_set_state_service_name = name
                    self.get_logger().info(f"切换目标物状态服务: {name}")
                return client

        return None

    def _set_entity_state_done_callback(self, future) -> None:
        self._set_state_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"更新目标物状态失败: {exc}", throttle_duration_sec=2.0)
            return

        if not response.success:
            self.get_logger().warn(f"目标物状态更新未成功: {response.status_message}", throttle_duration_sec=2.0)

    def _load_object_model_xml(self, model_path: str) -> str:
        try:
            with open(model_path, "r", encoding="utf-8") as model_file:
                return model_file.read()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"读取目标模型失败: {exc}")
            return ""

    def _try_delete_object_for_transport(self) -> None:
        use_fallback = self.get_parameter("use_delete_spawn_fallback").get_parameter_value().bool_value
        if not use_fallback or self._object_hidden_in_transport or self._delete_entity_in_flight:
            return
        if not self.delete_entity_client.service_is_ready():
            self.get_logger().warn("/delete_entity服务不可用，无法执行回退抓取", throttle_duration_sec=2.0)
            return

        req = DeleteEntity.Request()
        req.name = self.get_parameter("object_name").get_parameter_value().string_value
        self._delete_entity_in_flight = True
        future = self.delete_entity_client.call_async(req)
        future.add_done_callback(self._delete_entity_done_callback)

    def _delete_entity_done_callback(self, future) -> None:
        self._delete_entity_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"删除目标物失败: {exc}")
            return

        if response.success:
            self._object_hidden_in_transport = True
            self.get_logger().info("回退模式: 已删除目标物，模拟被夹取")
        else:
            self.get_logger().warn(f"删除目标物未成功: {response.status_message}")

    def _try_respawn_object_after_release(self) -> None:
        use_fallback = self.get_parameter("use_delete_spawn_fallback").get_parameter_value().bool_value
        if not use_fallback or not self._object_hidden_in_transport or self._spawn_entity_in_flight:
            return
        if not self.spawn_entity_client.service_is_ready():
            self.get_logger().warn("/spawn_entity服务不可用，无法执行回退放置", throttle_duration_sec=2.0)
            return
        if not self._object_model_xml:
            self.get_logger().warn("目标模型内容为空，无法重新生成目标物")
            return

        reference_frame = self.get_parameter("reference_frame").get_parameter_value().string_value
        ee_link = self.get_parameter("ee_link").get_parameter_value().string_value
        offset = self.get_parameter("attached_object_offset_xyz").get_parameter_value().double_array_value
        if len(offset) != 3:
            offset = [0.0, 0.0, 0.0]

        pose = Pose()
        try:
            transform = self.tf_buffer.lookup_transform(reference_frame, ee_link, rclpy.time.Time())
            pose.position.x = transform.transform.translation.x + float(offset[0])
            pose.position.y = transform.transform.translation.y + float(offset[1])
            pose.position.z = transform.transform.translation.z + float(offset[2])
            pose.orientation = transform.transform.rotation
        except TransformException:
            place = self._get_position_param("place_position")
            pose.position.x = place[0]
            pose.position.y = place[1]
            pose.position.z = place[2]
            pose.orientation.w = 1.0

        req = SpawnEntity.Request()
        req.name = self.get_parameter("object_name").get_parameter_value().string_value
        req.xml = self._object_model_xml
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = reference_frame

        self._spawn_entity_in_flight = True
        future = self.spawn_entity_client.call_async(req)
        future.add_done_callback(self._spawn_entity_done_callback)

    def _spawn_entity_done_callback(self, future) -> None:
        self._spawn_entity_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"重新生成目标物失败: {exc}")
            return

        if response.success:
            self._object_hidden_in_transport = False
            self.get_logger().info("回退模式: 已在放置位置重新生成目标物")
        else:
            self.get_logger().warn(f"重新生成目标物未成功: {response.status_message}")

    def _call_once_after(self, delay_sec: float, callback) -> None:
        holder = {}

        def _wrapped() -> None:
            timer = holder.get("timer")
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            callback()

        holder["timer"] = self.create_timer(max(delay_sec, 1e-3), _wrapped)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
