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


