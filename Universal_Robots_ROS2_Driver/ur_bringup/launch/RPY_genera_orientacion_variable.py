import numpy as np
import csv
from scipy.spatial.transform import Rotation as R

# Parámetros de la trayectoria circular
radio = 0.25
centro = np.array([-0.5, -0.5, 0.5])
num_puntos = 1000

# Punto fijo al que se apuntará
punto_fijo = np.array([-0.5, -0.5, 1.0])

# Generar los ángulos en radianes
theta = np.linspace(0, 2*np.pi, num_puntos)

# Calcular las coordenadas x, y, z de los puntos en la trayectoria circular
x = centro[0] + radio * np.cos(theta)
y = centro[1] + radio * np.sin(theta)
z = centro[2] * np.ones_like(theta)

# Combinar las coordenadas en una matriz de puntos
puntos = np.column_stack((x, y, z))

# Calcular la dirección hacia el punto fijo desde cada punto en la trayectoria
direcciones = punto_fijo - puntos

# Normalizar las direcciones para obtener vectores unitarios
direcciones_unitarias = direcciones / np.linalg.norm(direcciones, axis=1)[:, np.newaxis]

# Calcular los ángulos de Euler (roll, pitch, yaw) a partir de las direcciones
angulos_euler = np.zeros_like(direcciones_unitarias)
for i, direccion in enumerate(direcciones_unitarias):
    # Calcular el ángulo de yaw
    yaw = np.arctan2(direccion[1], direccion[0])
    # Calcular el ángulo de pitch
    pitch = np.arctan2(-direccion[2], np.sqrt(direccion[0]**2 + direccion[1]**2))
    # Roll se mantiene en 0 porque estamos en un movimiento circular
    angulos_euler[i] = [0, pitch, yaw]

# Escribir los puntos y las orientaciones en un archivo CSV
with open('/home/adela/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points_orientations.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for punto, euler_angles in zip(puntos, angulos_euler):
        writer.writerow([*punto, *euler_angles])

print("Archivo CSV generado correctamente.")
