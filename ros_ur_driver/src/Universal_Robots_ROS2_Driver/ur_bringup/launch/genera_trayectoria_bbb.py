import numpy as np
import csv

# Definir los límites para x e y
x_min, x_max = 0.2, 0.5
y_min, y_max = 0.3, 0.5

# Número de puntos para cada componente
num_puntos_linea = 300
num_puntos_semieje = 100

# Generar puntos para la línea recta
x_linea = np.linspace(x_min, x_max, num_puntos_linea)
y_linea = np.linspace(y_min, y_max, num_puntos_linea)

# Punto final de la línea recta
x_final_linea = x_linea[-1]
y_final_linea = y_linea[-1]

# Diámetros de las semicircunferencias
diametro = (x_max - x_min) / 2

# Generar puntos para las dos semicircunferencias
theta = np.linspace(-np.pi/2, np.pi/2, num_puntos_semieje)

x_semieje1 = x_max + diametro/2 * np.cos(theta)
y_semieje1 = y_max + diametro/2 * np.sin(theta)

x_semieje2 = x_min + diametro/2 * np.cos(theta)
y_semieje2 = y_min - diametro/2 + diametro/2 * np.sin(theta)

# Unir todos los segmentos para formar la trayectoria de la letra B
x = np.concatenate((x_linea, x_semieje1, x_semieje2))
y = np.concatenate((y_linea, y_semieje1, y_semieje2))

# Altura constante z=0.5
z = np.ones_like(x) * 0.5

# Guardar puntos en un archivo CSV
file_path = '/home/alvaro/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/config/points.csv'
with open(file_path, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for i in range(len(x)):
        writer.writerow([x[i], y[i], z[i]])

print("Archivo CSV generado correctamente en la ubicación especificada.")
