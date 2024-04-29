# Este paquete permite trazar y representar trayectorias cartesianas sencillas devolviendo sus coordenadas cartesianas

#  Se importan las librerias necesarias
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os 
import datetime
import csv
def castillo(n_puntos=1000, delta_x=0.1,  delta_y=0.5, file_path='/home/adela/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/castillo.csv'):
    # CASTILLO
    x = np.linspace(-0.75, -0.25, n_puntos)  # Coordenada x que avanza linealmente
    z = np.zeros_like(x) + 0.75      # Cota z constante de 10 unidades

    # Coordenada y: onda cuadrada similar en el plano XY
    y_range = 4 - 2
    y_offset = -0.5
    y = delta_y * np.sign(np.sin(x / delta_x * np.pi)) * (y_range / 2) + y_offset
    #y = delta_y * np.sign(np.sin(x / delta_x * np.pi))  # Amplitud de 20 unidades, periodo de 10 unidades

    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
       
        for point in zip(x, y, z):
            writer.writerow(point)
            
    return x,y,z

castillo()

def muelle(n_puntos=500, n_vueltas=5, radio=4):
    t = np.linspace(0, 2*n_vueltas*np.pi, n_puntos)  # Ángulo que varía desde 0 hasta 10*pi
    x = np.linspace(0, 2, n_puntos)             # Longitud que varía desde 0 hasta 1
    z = radio * np.sin(t)                   # Coordenada x en función de t
    y = radio * np.cos(t)                   # Coordenada y en función de t

    return x,y,z

def muelle_sensual(n_puntos=500, n_vueltas=5, r_max=4, r_min=2 ,longitud=2):
    t = np.linspace(0, n_vueltas*2*np.pi, n_puntos)  # Ángulo que varía desde 0 hasta 10*pi
    x = np.linspace(0, longitud, n_puntos)             # Longitud que varía desde 0 hasta 1
    A=(r_max-r_min)*4/longitud**2
    radio=A*(x-longitud/2)**2+r_min
    z = radio * np.sin(t)                   # Coordenada x en función de t
    y = radio * np.cos(t)                   # Coordenada y en función de t

    return x,y,z

def muelle_abonbado(n_puntos=500, n_vueltas=5, r_max=4, r_min=2 ,longitud=2):
    t = np.linspace(0, n_vueltas*2*np.pi, n_puntos)  # Ángulo que varía desde 0 hasta 10*pi
    x = np.linspace(0, longitud, n_puntos)             # Longitud que varía desde 0 hasta 1
    A= (r_min-r_max)*4/longitud**2
    B= r_max
    radio=A*(x-longitud/2)**2+B
    z = radio * np.sin(t)                   # Coordenada x en función de t
    y = radio * np.cos(t)                   # Coordenada y en función de t

    return x,y,z


def vueltas_plano_xy(n_puntos=500, n_vueltas=5, altitud=3, radio=4):
    t = np.linspace(0, n_vueltas*2*np.pi, n_puntos)  # Ángulo que varía desde 0 hasta 10*pi
    z = altitud             # Longitud que varía desde 0 hasta 1
    y = radio * np.sin(t)                   # Coordenada x en función de t
    x = radio * np.cos(t)                   # Coordenada y en función de t

    return x,y,z