import math, csv, random
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

S = math.sqrt(0.5)
ORIENTATIONS = {            # tool z-axis pointing: down, up, +x, -x, +y, -y
    "down": (1, 0, 0, 0), "up": (0, 1, 0, 0),
    "+x": (0, S, 0, S),   "-x": (0, -S, 0, S),
    "+y": (-S, 0, 0, S),  "-y": (S, 0, 0, S),
}
total_false = 0
total_true = 0
N_POINTS = 1000  # change to 300 if you have time

rclpy.init()
node = Node("reach_study")
cli = node.create_client(GetPositionIK, "/compute_ik")
cli.wait_for_service()
random.seed() 

rows = []
for i in range(N_POINTS):
    r = random.uniform(0.2, 0.9)
    az = random.uniform(-math.pi, math.pi)
    el = random.uniform(-math.pi/4, math.pi/2)
    x, y = r*math.cos(el)*math.cos(az), r*math.cos(el)*math.sin(az)
    z = 0.333 + r*math.sin(el)          
    ok_any = False
    for name, q in ORIENTATIONS.items():
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
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut)
        if fut.result().error_code.val == 1:   # SUCCESS
            ok_any = True
            break
    rows.append([i, x, y, z, r, az, el, ok_any])
    if ok_any:
        total_true += 1
    else:
        total_false += 1
    print(f"{i+1}/{N_POINTS}  reachable={ok_any}  | True so far: {total_true}  False so far: {total_false}", flush=True)

with open("reach.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["id","x","y","z","r","az","el","reachable"])
    w.writerows(rows)

print("Done. Wrote reach.csv")
rclpy.shutdown()
