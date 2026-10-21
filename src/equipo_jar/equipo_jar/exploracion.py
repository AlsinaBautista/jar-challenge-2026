import math

def yaw_de_cuaternion(q):
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return yaw

def diferencia_angulo(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

def ya_visitado(x, y, yaw, visitados, dist_min, ang_min):
    for tup in visitados:
        xi = tup[0]
        yi = tup[1]
        dx = x - xi
        dy = y - yi
        yawi = tup[2]
        dist = math.hypot(dx, dy)
        ang = diferencia_angulo(yaw, yawi)
        if dist < dist_min and ang < ang_min:
            return True
    return False

def primer_pendiente(poses, visitados, dist_min, ang_min):
    for p in poses:
        x = p.pose.position.x
        y = p.pose.position.y
        q = p.pose.orientation
        yaw = yaw_de_cuaternion(q)
        visitado = ya_visitado(x, y, yaw, visitados, dist_min, ang_min)
        if not visitado:
            return p
    return None
