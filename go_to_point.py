import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


class Reach(Node):
    def __init__(self):
        super().__init__("reach")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.mg = ActionClient(self, MoveGroup, "/move_action")
        self.ik.wait_for_service()
        self.mg.wait_for_server()

    def go(self, x, y, z, quat=(1.0, 0.0, 0.0, 0.0)):  # quat (1,0,0,0): tool points down
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_link8"
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        p = PoseStamped()
        p.header.frame_id = "panda_link0"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        (p.pose.orientation.x, p.pose.orientation.y,
         p.pose.orientation.z, p.pose.orientation.w) = quat
        req.ik_request.pose_stamped = p

        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res.error_code.val != 1:
            self.get_logger().warn(f"No IK solution (code {res.error_code.val})")
            return False

        js = res.solution.joint_state
        c = Constraints()
        for n, pos in zip(js.name, js.position):
            if n.startswith("panda_joint"):
                c.joint_constraints.append(JointConstraint(
                    joint_name=n, position=pos,
                    tolerance_above=0.001, tolerance_below=0.001, weight=1.0))

        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.goal_constraints = [c]
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.planning_options.plan_only = False   # True = plan without moving

        gf = self.mg.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gf)
        handle = gf.result()
        if not handle.accepted:
            return False
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        ok = rf.result().result.error_code.val == 1
        self.get_logger().info("Reached" if ok else "Planning/execution failed")
        return ok


rclpy.init()
node = Reach()
x, y, z = (float(v) for v in sys.argv[1:4]) if len(sys.argv) >= 4 else (0.4, 0.0, 0.4)
node.go(x, y, z)
rclpy.shutdown()
