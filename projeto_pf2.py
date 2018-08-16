#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles, draw_random_sample
import numpy as np
import inspercles # necessário para o a função nb_lidar que simula o laser
import math
from scipy.stats import norm
import mpmath


largura = 775 # largura do mapa
altura = 748  # altura do mapa

# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)

# Nuvem de particulas
particulas = []

num_particulas = 10


# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=16, endpoint=False)

# Lista mais longa
movimentos_longos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0],
              [0,0,math.pi/12.0], [0, 0, math.pi/12.0], [0, 0, math.pi/12],[0,0,-math.pi/4],
              [-5, 0, 0],[-5,0,0], [-5,0,0], [-10,0,0],[-10,0,0], [-10,0,0],[-10,0,0],[-10,0,0],[-15,0,0],
              [0,0,-math.pi/4],[0, 10, 0], [0,10,0], [0, 10, 0], [0,10,0], [0,0,math.pi/8], [0,10,0], [0,10,0], 
              [0,10,0], [0,10,0], [0,10,0],[0,10,0],
              [0,0,-math.radians(90)],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0]]

# Lista curta
movimentos_curtos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0]]

movimentos_relativos = [[0, -math.pi/3],[10, 0],[10, 0], [10, 0], [10, 0],[15, 0],[15, 0],[15, 0],[0, -math.pi/2],[10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [0, -math.pi/2], 
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [0, -math.pi/2], 
                       [10,0], [0, -math.pi/4], [10,0], [10,0], [10,0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0], [40,0], [0, math.pi/3]]



movimentos = movimentos_relativos

std_mov = 0.1
std_theta = math.radians(0.5)
std_laser =2
std_resample_x = 3
std_resample_y = 3
std_resample_theta = math.radians(3)
delta_cdf = std_laser/3.0# Delta usado na CDF


def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas):
    """
        Cria uma lista de partículas distribuídas de forma uniforme entre minx, miny, maxx e maxy
    """
    particle_cloud = []
    for i in range(n_particulas):
        x = np.random.uniform(minx, maxx)
        y = np.random.uniform(miny, maxy)
        theta = np.random.uniform(0, 2*math.pi)
        p = Particle(x, y, theta, w=1.0) # A prob. w vai ser normalizada depois
        particle_cloud.append(p)
    return particle_cloud
    
    
def move_particulas(particulas, movimento):
    """
        Recebe um movimento na forma [deslocamento, theta]  e o aplica a todas as partículas
        Assumindo um desvio padrão para cada um dos valores
        Esta função não precisa devolver nada, e sim alterar as partículas recebidas.
        
        Sugestão: aplicar move_relative(movimento) a cada partícula
        
        Você não precisa mover o robô. O código fornecido pelos professores fará isso
        
    """
    for p in particulas:
        p.move_relative([movimento[0] + norm.rvs(std_mov), movimento[1] + norm.rvs(std_theta)])
        
    return particulas
    
def pdh_area_sum(lr, lpart):
    """
        lr - leitura do robô
        lpart - leitura da partícula
    """
    pdh= 0.0
    for a in lr:
        cdfs = norm.cdf([lpart[a] - delta_cdf, lpart[a] + delta_cdf],loc=lr[a], scale=std_laser)
        delta_prob = cdfs[1] = cdfs[0]
        pdh+=delta_prob
    return pdh


def pdh_sum(lr, lpart):
    """
        lr - leitura do robô
        lpart - leitura da partícula
    """
    pdh= 0.0
    for a in lr:
        prob = norm.pdf(lpart[a], loc=lr[a], scale=std_laser)
        pdh+=prob
    return pdh

def pdh_prod_mpf(lr, lpart):
    pdh = mpmath.mpf(1.0)
    for a in lr:
        prob = norm.pdf(lpart[a], loc=lr[a], scale=std_laser)
        pdh*=prob
    return pdh

    

def leituras_laser_evidencias_plain(robot, particulas):
    """
        Realiza leituras simuladas do laser para o robo e as particulas
        Depois incorpora a evidência calculando
        P(H|D) para todas as particulas
        Lembre-se de que a formula $P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$ 
        responde somente P(Hi|D), em que H é a hi
        
        Esta função não precisa retornar nada, mas as partículas precisa ter o seu w recalculado. 
        
        Você vai precisar calcular para o robo
        
    """
    isfast = False
    
    leitura_robo = inspercles.nb_lidar(robot, angles,fast=isfast)
    
    soma = 0.0
    
    for p in particulas:
        leitura_part = inspercles.nb_lidar(p, angles, fast=isfast)
        pdh = pdh_sum(leitura_robo, leitura_part)**2
        p.w = pdh
        soma+=pdh

    for p in particulas:
        if soma > 0.0:        
            p.w/=soma
            p.w = float(p.w)
        else:
            p.w = 1.0

def leituras_laser_evidencias_sum(robot, particulas):
    """
        Realiza leituras simuladas do laser para o robo e as particulas
        Depois incorpora a evidência calculando
        P(H|D) para todas as particulas
        Lembre-se de que a formula $P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$ 
        responde somente P(Hi|D), em que H é a hi
        
        Esta função não precisa retornar nada, mas as partículas precisa ter o seu w recalculado. 
        
        Você vai precisar calcular para o robo
        
    """
    isfast = True
    
    leitura_robo = inspercles.nb_lidar(robot, angles,fast=isfast)
    
    soma = 0.0
    
    for p in particulas:
        leitura_part = inspercles.nb_lidar(p, angles, fast=isfast)
        pdh = pdh_sum(leitura_robo, leitura_part)
        p.w = pdh
        soma+=pdh

    for p in particulas:
        if soma > 0.0:        
            p.w/=soma
            p.w = float(p.w)
        else:
            p.w = 1.0

def leituras_laser_evidencias_produto(robot, particulas):
    """
        Realiza leituras simuladas do laser para o robo e as particulas
        Depois incorpora a evidência calculando
        P(H|D) para todas as particulas
        Lembre-se de que a formula $P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$ 
        responde somente P(Hi|D), em que H é a hi
        
        Esta função não precisa retornar nada, mas as partículas precisa ter o seu w recalculado. 
        
        Você vai precisar calcular para o robo
        
    """
    isfast = False
    
    leitura_robo = inspercles.nb_lidar(robot, angles,fast=isfast)
    
    soma = mpmath.mpf(0.0)
    
    for p in particulas:
        leitura_part = inspercles.nb_lidar(p, angles, fast=isfast)
        pdh = pdh_prod_mpf(leitura_robo, leitura_part)
        p.w = pdh
        soma+=pdh

    for p in particulas:
        if soma > 0.0:        
            p.w/=soma
            p.w = float(p.w)
        else:
            p.w = 1.0



def reamostrar(particulas, n_particulas = num_particulas):
    """
        Reamostra as partículas devolvendo novas particulas sorteadas
        de acordo com a probabilidade e deslocadas de acordo com uma variação normal    
        
        O notebook como_sortear tem dicas que podem ser úteis
        
        Depois de reamostradas todas as partículas precisam novamente ser deixadas com probabilidade igual
        
        Use 1/n ou 1, não importa desde que seja a mesma
    """
    probs = [p.w for p in particulas]

    print("Probabilidades: ")
    print(probs)
    print("Soma probs")
    print(sum(probs))

    pfinal = draw_random_sample(particulas, probs, n_particulas)
    
    
    for p in pfinal:
        p.x+=norm.rvs(scale=std_resample_x)
        p.y+=norm.rvs(scale=std_resample_y)
        p.theta+=norm.rvs(scale=std_resample_theta)
        p.w = 1.0

    return pfinal


    







