import math

def desvio_maximo(cov_xy):
    #cov_xy = [a, b, b, d] matriz de covarianza escrita como vector
    a = cov_xy[0] 
    b = cov_xy[1]
    d = cov_xy[3]
    lambda_max = (a + d) / 2 + math.sqrt(((a - d) / 2) ** 2 + b ** 2) 
    return math.sqrt(max(lambda_max, 0.0))

def esta_confirmada(obs, cov_xy, min_obs, max_desvio):
    if obs >= min_obs and desvio_maximo(cov_xy) <= max_desvio:
        return True
    return False

def cerca_de_alguno(x, y, puntos, dist_min):
    for p in puntos:
        xi, yi = p
        dx = abs(x - xi)
        dy = abs(y - yi)
        dist = math.hypot(dx, dy)
        if dist <= dist_min:
            return True
    return False

def elegir_para_reportar(cands, ids_reportados, puntos_reportados, n, min_obs, max_desvio, dist_min):
    quedan = n - len(puntos_reportados)
    if quedan <= 0:
        return []
    prioridad = sorted(cands, key=lambda c: desvio_maximo(c.covarianza_xy))
    elegidas = []
    puntos_elegidos = []
    for c in prioridad:
        confirmada = esta_confirmada(c.observaciones, c.covarianza_xy, min_obs, max_desvio)
        if ((not confirmada) or (c.id in ids_reportados) 
            or cerca_de_alguno(c.centro.x, c.centro.y, puntos_reportados + puntos_elegidos, dist_min)
            or (not math.isfinite(c.centro.x)) or (not math.isfinite(c.centro.y))):
            continue
        elegidas.append(c)
        puntos_elegidos.append((c.centro.x, c.centro.y))
        if len(elegidas) == quedan:
            break
    return elegidas
