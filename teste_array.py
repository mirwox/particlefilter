#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 29 15:52:17 2018

@author: mirwox
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
from random import randint, choice
import time
import matplotlib.patches as mpatches
import numpy as np
import math
import random
from pf import Particle
import rayline
from intersection.intersection import find_intersections
from intersection.segment import Segment
from time import time
import projeto_pf
import inspercles


lines = None

map_name = 'sparse_obstacles'
img = cv2.imread(map_name + '.png')
with open(map_name + '_lines.txt', 'r') as f:
    segments = np.array([[float(j) for j in line.split()] for line in f])

print("Segments",segments.shape)
print(segments)


occupied_thresh = 0.2
free_thresh = 0.9


map_name = 'sparse_obstacles'
img = cv2.imread(map_name + '.png')

minLineLength=4


def canny_lines(img):
    """
        Retorna todos os segmentos de linha contidos numa imagem
    """
    np_image = img
    canny = cv2.Canny(np_image, occupied_thresh*255, free_thresh*255)
    #kernel = np.ones((5,5), np.uint8)
    #canny = cv2.dilate(canny, kernel, iterations=1)
    linhas = np.array([])
    lines = cv2.HoughLinesP(image=canny,rho=0.02,theta=np.pi/1000, threshold=25,lines=linhas, minLineLength=minLineLength,maxLineGap=3)
    return lines

    
#lines = segments

def real_angle(p1, p2):
    return math.atan2((p2[1] - p1[1]) , (p2[0] - p1[0]))

if lines is None:
    lines = canny_lines(img)

print("Lines shape", lines.shape)
print(lines)


print("SEGMENTS")
for item in segments:
    print("\t", item)
    
print("LINES")
for item in lines:
    print("\t",item)
    

angles = projeto_pf.angles

canvas       = img.copy()
canvas_angle = img.copy()


# Começando com o nb_lidar2

particle = projeto_pf.robot
particle.theta = math.pi/4


lines = segments

directions = inspercles.make_directions(particle, angles)
origin = (particle.x, particle.y)
interpoints = inspercles.closest_intersections(origin, directions, lines)
dists = []
for p in interpoints:
  dist = math.sqrt((p[0]-origin[0])**2 + (p[1] - origin[1])**2)
  dists.append(dist)
readings= dict(zip(angles, dists))

origin = (int(particle.x), int(particle.y))

deltac = 0

font = cv2.FONT_HERSHEY_PLAIN

i = 0
for p in interpoints:
    dst = (int(p[0]), int(p[1]))
    linha_baixo = (dst[0]+10, dst[1]-20)
    cv2.line(canvas, origin, dst,  (deltac,deltac,deltac),3, lineType=cv2.LINE_AA)
    ra = real_angle(origin, dst)
    cv2.putText(canvas, "%d a %4.2f ra %4.2f d %4.2f"%(i,angles[i], ra, dists[i]), dst,font, 1, (255,0,0),2, lineType=cv2.LINE_AA)
    cv2.putText(canvas, "dif %4.2f"%(ra - angles[i]), linha_baixo,font, 1, (0,255,0),2, lineType=cv2.LINE_AA)
    deltac += 25
    i+=1

plt.imshow(canvas)

cv2.imwrite( "teste_array.png", canvas)

leituras = inspercles.nb_lidar(particle, projeto_pf.angles)


# WARNING: o nb_lidar antigo soma o ângulo da pose do robô nos ângulos relativos dos sensores
# Foi renomeado para nb_lidar_old

# O nblidar2 está ok!!! PAssou a ser o nb_lidar oficial

deltac = 0
for a in sorted(leituras.keys()):
    l = leituras[a]
    dx = l*math.cos(a + particle.theta) 
    dy = l*math.sin(a + particle.theta)
    endpoint = (int(particle.x + dx), int(particle.y + dy))
    cv2.line(canvas_angle, origin, endpoint,  (deltac,deltac,deltac),3, lineType=cv2.LINE_AA)
    cv2.putText(canvas_angle, "%4.2f %4.2f"%(a,l), endpoint,font, 1, (0,0,255),2, lineType=cv2.LINE_AA)
    
    deltac += 25

cv2.imwrite("teste_array_canvas_angle.png", canvas_angle)
plt.imshow(canvas_angle)
    
    


    
    



