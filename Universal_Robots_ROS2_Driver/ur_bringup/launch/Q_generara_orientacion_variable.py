import numpy as np
import csv

# Parámetros de la trayectoria circular
radio = 0.25
centro = np.array([0.25, 0.5, 1.5])
num_puntos = 1000

# Punto fijo al que se apuntará
punto_fijo = np.array([0.25, 0.5, 2.0])

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

# Calcular el ángulo de rotación necesario en radianes
angulo_rotacion = np.arccos(np.dot([0, 0, 1], direcciones_unitarias.T))

# Calcular el componente w de la orientación
orientation_w = np.cos(angulo_rotacion / 2)


# Escribir los puntos y las orientaciones en un archivo CSV
with open('/home/adela/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points_quaternion.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for punto, direccion, w in zip(puntos, direcciones_unitarias, orientation_w):
        writer.writerow([*punto, *direccion, w])

print("Archivo CSV generado correctamente.")