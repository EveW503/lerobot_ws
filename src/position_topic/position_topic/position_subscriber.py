import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import MotionPlanRequest
# 已经不需要 OrientationConstraint 了，可以去掉或者保留import
from moveit_msgs.msg import OrientationConstraint 
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import PositionConstraint
from shape_msgs.msg import SolidPrimitive


class NodeSubscriber(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.get_logger().info("目标位姿订阅节点已启动 (当前为: 仅位置约束模式)")

        self.declare_parameter("topic_name", "/target_pose")
        self.declare_parameter("action_name", "/move_action")
        self.declare_parameter("group_name", "arm")
        self.declare_parameter("ee_link", "gripper")
        self.declare_parameter("reference_frame", "base")
        self.declare_parameter("pipeline_id", "")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("max_velocity_scaling_factor", 0.3)
        self.declare_parameter("max_acceleration_scaling_factor", 0.3)
        self.declare_parameter("position_tolerance", 0.01) # 位置允许误差，默认1cm
        self.declare_parameter("execute_trajectory", True)
        self.declare_parameter("replan", False)
        self.declare_parameter("replan_attempts", 3)
        self.declare_parameter("replan_delay", 0.5)
        self.declare_parameter("cancel_previous_goal", True)
        self.declare_parameter("wait_for_server_timeout_sec", 5.0)

        topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        action_name = self.get_parameter("action_name").get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.target_pose_callback,
            10,
        )

        self.action_client = ActionClient(self, MoveGroup, action_name)
        wait_timeout = self.get_parameter("wait_for_server_timeout_sec").get_parameter_value().double_value
        self._server_ready = self.action_client.wait_for_server(timeout_sec=wait_timeout)
        if self._server_ready:
            self.get_logger().info(f"已连接move_group action: {action_name}")
        else:
            self.get_logger().warn(
                f"未在{wait_timeout:.1f}s内连接到{action_name}，后续收到目标时会重试检查"
            )

        self._active_goal_handle = None
        self._pending_target = None

    def target_pose_callback(self, msg: PoseStamped) -> None:
        self.get_logger().info(
            "收到目标位置: "
            f"frame={msg.header.frame_id}, "
            f"p=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) [忽略姿态]"
        )

        if not self.action_client.server_is_ready():
            self.get_logger().warn("move_group action当前不可用，跳过本次目标")
            return

        cancel_previous = self.get_parameter("cancel_previous_goal").get_parameter_value().bool_value
        if self._active_goal_handle is not None and cancel_previous:
            self._pending_target = msg
            self.get_logger().info("检测到正在执行的目标，先取消旧目标再发送新目标")
            cancel_future = self._active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            return

        self.send_goal_to_move_group(msg)

    def send_goal_to_move_group(self, msg: PoseStamped) -> None:
        goal = MoveGroup.Goal()
        goal.request = self._build_motion_plan_request(msg)
        goal.planning_options = self._build_planning_options()

        self.get_logger().info("发送MoveGroup目标中 (仅进行3D位置规划)...")
        send_future = self.action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_callback)

    def _build_motion_plan_request(self, target: PoseStamped):
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

        frame_id = target.header.frame_id or self.get_parameter("reference_frame").get_parameter_value().string_value
        ee_link = self.get_parameter("ee_link").get_parameter_value().string_value
        position_tolerance = self.get_parameter("position_tolerance").get_parameter_value().double_value

        if not ee_link:
            self.get_logger().warn("参数ee_link为空，目标很可能无法被MoveIt识别")

        constraints = Constraints()

        # ========== 核心修改：只保留位置约束 (Position Constraint) ==========
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = ee_link
        pos_constraint.weight = 1.0

        # 创建一个球形目标区域，半径为设定的容差
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [max(position_tolerance, 1e-4)]

        region = BoundingVolume()
        region.primitives.append(sphere)

        target_pose = Pose()
        target_pose.position = target.pose.position
        target_pose.orientation.w = 1.0 # 仅作为占位符
        region.primitive_poses.append(target_pose)
        pos_constraint.constraint_region = region

        # 将位置约束加入总约束列表中
        constraints.position_constraints.append(pos_constraint)
        
        # 删除(屏蔽)了之前构建和添加 OrientationConstraint 的所有代码
        # ----------------------------------------------------------------

        request.goal_constraints.append(constraints)
        return request

    def _build_planning_options(self):
        planning_options = PlanningOptions()
        execute_trajectory = self.get_parameter("execute_trajectory").get_parameter_value().bool_value
        planning_options.plan_only = not execute_trajectory
        planning_options.look_around = False
        planning_options.look_around_attempts = 0
        planning_options.max_safe_execution_cost = 0.0
        planning_options.replan = self.get_parameter("replan").get_parameter_value().bool_value
        planning_options.replan_attempts = self.get_parameter("replan_attempts").get_parameter_value().integer_value
        planning_options.replan_delay = self.get_parameter("replan_delay").get_parameter_value().double_value
        return planning_options

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"发送MoveGroup目标失败: {exc}")
            self._active_goal_handle = None
            return

        if not goal_handle.accepted:
            self.get_logger().warn("MoveGroup拒绝了该目标")
            self._active_goal_handle = None
            return

        self._active_goal_handle = goal_handle
        self.get_logger().info("MoveGroup已接受目标，等待规划/执行结果")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"获取MoveGroup结果失败: {exc}")
            self._active_goal_handle = None
            return

        status = wrapped_result.status
        result = wrapped_result.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"MoveGroup成功，planning_time={result.planning_time:.3f}s, error_code={result.error_code.val}"
            )
        else:
            self.get_logger().warn(
                f"MoveGroup结束状态={status}, error_code={result.error_code.val}, planning_time={result.planning_time:.3f}s"
            )

        self._active_goal_handle = None

    def _cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"取消目标失败: {exc}")
            self._active_goal_handle = None
            return

        cancel_count = len(cancel_response.goals_canceling)
        self.get_logger().info(f"取消请求已返回，goals_canceling数量={cancel_count}")
        self._active_goal_handle = None

        if self._pending_target is not None:
            pending = self._pending_target
            self._pending_target = None
            self.send_goal_to_move_group(pending)


def main(args=None):
    rclpy.init(args=args)
    node = NodeSubscriber("target_pose_subscriber")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()