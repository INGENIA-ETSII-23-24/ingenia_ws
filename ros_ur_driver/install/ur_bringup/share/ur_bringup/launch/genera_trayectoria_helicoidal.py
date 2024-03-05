import numpy as np
import csv

# Parámetros de la trayectoria helicoidal
radio = 0.1  # Radio del círculo en metros
altura_inicial = 1.0  # Altura inicial en metros
descenso_por_vuelta = 0.05  # Descenso de altura por vuelta en metros
num_puntos_por_vuelta = 1000  # Número de puntos en cada vuelta

# Coordenadas del punto inicial
x0, y0, z0 = 0.5, 0.5, 1.0

# Función para calcular la altura en función del ángulo
def calcular_altura(theta):
    vueltas_completas = theta / (2 * np.pi)
    altura = max(altura_inicial - vueltas_completas * descenso_por_vuelta, 0)
    return altura

# Generar puntos en la trayectoria helicoidal
theta = np.linspace(0, 20 * np.pi, num_puntos_por_vuelta * 20)  # Ángulos en radianes
z = np.array([calcular_altura(t) for t in theta])  # Altura descendente
x = x0 + radio * np.cos(theta)  # Coordenadas x
y = y0 + radio * np.sin(theta)  # Coordenadas y

# Guardar puntos en un archivo CSV
with open('/home/alvaro/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for i in range(len(x)):
        writer.writerow([x[i], y[i], z[i]])

print("Archivo CSV generado correctamente.")

