#ESTE PROGRAMA SE ENCARGA DE GENERAL UNA TRAYECTORIA CIRCULAR Y ESCRIBIR LOS PUNTOS EN UN .CSV PARA PODER
#PROBAR LOS DIFERENTES PROGRAMAS DESARROLLADOS.

#PARA LANZARLA, SE USA EL PROGRAMA MOVERJ
import numpy as np
import csv

# Parámetros de la trayectoria circular. MODIFICAR SEGÚN SE DESEE.
radio = 0.25
centro = np.array([-0.5, -0.5, 0.5])
num_puntos = 1000000

# Generar los ángulos en radianes
theta = np.linspace(0, 2*np.pi, num_puntos)

# Calcular las coordenadas x, y, z de los puntos en la trayectoria circular
x = centro[0] + radio * np.cos(theta)
y = centro[1] + radio * np.sin(theta)
z = centro[2] * np.ones_like(theta)

# Combinar las coordenadas en una matriz de puntos
puntos = np.column_stack((x, y, z))

# Escribir los puntos en un archivo CSV
with open('/home/adela/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerows(puntos)

print("Archivo CSV generado correctamente.")
