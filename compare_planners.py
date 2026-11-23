import math, csv
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

S = math.sqrt(0.5)
ORIENTATIONS = [(1,0,0,0), (0,1,0,0), (0,S,0,S), (0,-S,0,S), (-S,0,0,S), (S,0,0,S)]
PLANNERS = ["RRTConnectkConfigDefault", "BiTRRTkConfigDefault"]
REPEATS = 3
MAX_POINTS = 100      # raise to 50 if time allows
PLAN_TIME = 5.0
NAN = float("nan")

rclpy.init()
node = Node("compare_planners")
ik = node.create_client(GetPositionIK, "/compute_ik")
mg = ActionClient(node, MoveGroup, "/move_action")
ik.wait_for_service()
mg.wait_for_server()

def solve_ik(x, y, z):
    for q in ORIENTATIONS:
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_link8"
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 1
        p = PoseStamped()
        p.header.frame_id = "panda_link0"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
        (p.pose.orientation.x, p.pose.orientation.y,
         p.pose.orientation.z, p.pose.orientation.w) = (float(v) for v in q)
        req.ik_request.pose_stamped = p
        fut = ik.call_async(req)
        rclpy.spin_until_future_complete(node, fut)
        res = fut.result()
        if res.error_code.val == 1:
            js = res.solution.joint_state
            return [(n, v) for n, v in zip(js.name, js.position)
                    if n.startswith("panda_joint")]
    return None

def tsec(p):
    return p.time_from_start.sec + p.time_from_start.nanosec * 1e-9

def plan(joints, planner):
    c = Constraints()
    for n, v in joints:
        c.joint_constraints.append(JointConstraint(
            joint_name=n, position=float(v),
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
    goal = MoveGroup.Goal()
    goal.request.group_name = "panda_arm"
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = planner
    goal.request.goal_constraints = [c]
    goal.request.num_planning_attempts = 1
    goal.request.allowed_planning_time = PLAN_TIME
    goal.planning_options.plan_only = True
    gf = mg.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, gf)
    handle = gf.result()
    if not handle.accepted:
        return False, NAN, NAN, NAN
    rf = handle.get_result_async()
    rclpy.spin_until_future_complete(node, rf)
    r = rf.result().result
    if r.error_code.val != 1:
        return False, r.planning_time, NAN, NAN
    pts = r.planned_trajectory.joint_trajectory.points
    length = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        length += math.sqrt(sum((u - v) ** 2 for u, v in zip(a.positions, b.positions)))
    jerks = []
    for a, b in zip(pts[:-1], pts[1:]):
        dt = tsec(b) - tsec(a)
        if dt > 1e-6 and a.accelerations and b.accelerations:
            for ua, ub in zip(a.accelerations, b.accelerations):
                jerks.append(((ub - ua) / dt) ** 2)
    jerk = math.sqrt(sum(jerks) / len(jerks)) if jerks else NAN
    return True, r.planning_time, length, jerk

with open("reach.csv") as f:
    points = [r for r in csv.DictReader(f) if r["reachable"] == "True"][:MAX_POINTS]
print(f"Using {len(points)} reachable points", flush=True)

count1 = {p: {"true": 0, "false": 0, "time_ok": 0.0, "time_all": 0.0} for p in PLANNERS}
count = {p: {"true": 0, "false": 0} for p in PLANNERS}
rows = []
for k, pt in enumerate(points):
    joints = solve_ik(float(pt["x"]), float(pt["y"]), float(pt["z"]))
    if joints is None:
        continue
    for planner in PLANNERS:
        for rep in range(REPEATS):
            ok, t, length, jerk = plan(joints, planner)
            rows.append([pt["id"], planner, rep, ok, t, length, jerk])
            count[planner]["true" if ok else "false"] += 1
            if t == t:  # skips NaN
                count1[planner]["time_all"] += t
                if ok:
                    count1[planner]["time_ok"] += t
            tally = "  |  ".join(
                f"{p.replace('kConfigDefault','')}: T={count[p]['true']} F={count[p]['false']}"
                for p in PLANNERS)
            print(f"pointt {k+1}/{len(points)} repition {rep}  {planner.replace('kConfigDefault','')} ok={ok} "
                f"t={round(t,3)}  ||  {tally}", flush=True)

with open("planner_results.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["point_id","planner","rep","success","planning_time_s","path_length_rad","rms_jerk"])
    w.writerows(rows)
print("Done. Wrote planner_results.csv")
for p in PLANNERS:
    tr, fa = count[p]["true"], count[p]["false"]
    total = tr + fa
    rate = 100 * tr / total if total else 0
    avg_ok = count1[p]["time_ok"] / tr if tr else float("nan")
    avg_all = count1[p]["time_all"] / total if total else float("nan")
    
    print(f"{p}: success={tr}  failure={fa}  total={total}  success rate={rate:.1f}%")
    print(f"averagetime (successful)={avg_ok:.3f}s average time (all)= {avg_all:.3f}s")
rclpy.shutdown()
