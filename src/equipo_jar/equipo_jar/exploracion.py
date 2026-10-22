import math

def yaw_from_quaternion(q):
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return yaw

def angle_difference(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

def already_visited(x, y, yaw, visited, dist_min, ang_min):
    for tup in visited:
        xi = tup[0]
        yi = tup[1]
        dx = x - xi
        dy = y - yi
        yawi = tup[2]
        dist = math.hypot(dx, dy)
        ang = angle_difference(yaw, yawi)
        if dist < dist_min and ang < ang_min:
            return True
    return False

def first_pending(poses, visited, dist_min, ang_min):
    for p in poses:
        x = p.pose.position.x
        y = p.pose.position.y
        q = p.pose.orientation
        yaw = yaw_from_quaternion(q)
        seen = already_visited(x, y, yaw, visited, dist_min, ang_min)
        if not seen:
            return p
    return None
