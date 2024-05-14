#Programa que se suscribe al nodo "micro_ros_arduino_node", creado por una ESP32,
#  e intercambia información, como temperatura objetivo del extrusor, si hay  que
# imprimir o no, que temperatura tiene el extrusor, y si se esta imprimiendo o no.
# Para entenderlo mejor ver el archivo micro-ros_publisher_prueba.ino en el paquete micro_ros, 
# en la siguiente dirección:

# /workspace/ros_ur_driver/src/micro_ros_arduino/examples/micro-ros_publisher_prueba/micro-ros_publisher_prueba.ino


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class HumbleNode(Node):
    def __init__(self):
        super().__init__('micro_ros_arduino_node')
        self.publisher_temperatura_objetivo = self.create_publisher(Int32, 'temperatura_objetivo', 10)
        self.publisher_temperatura_cama_objetivo = self.create_publisher(Int32, 'temperatura_cama_objetivo', 10)
        self.publisher_imprimir_objetivo = self.create_publisher(Int32, 'imprimir_objetivo', 10)
        self.publisher_longitud_extrusion_objetivo = self.create_publisher(Int32, 'longitud_extrusion_objetivo', 10)
        
        self.subscription_temperatura = self.create_subscription(Int32, 'temperatura', self.temperatura_callback, 10)
        self.subscription_temperatura_cama = self.create_subscription(Int32, 'temperatura_cama', self.temperatura_cama_callback, 10)
        self.subscription_imprimir = self.create_subscription(Int32, 'imprimir', self.imprimir_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
#DATOS QUE SE QUIEREN ENVIAR ----------------------------------------------------------------------------------------------------------    
        
        self.temperatura_objetivo = 200
        self.temperatura_cama_objetivo = 60
        self.imprimir_objetivo = 1
        self.longitud_extrusion_objetivo = 1000
        
#--------------------------------------------------------------------------------------------------------------------------------------

    def timer_callback(self):
        temperatura_msg = Int32()
        temperatura_cama_msg = Int32()
        imprimir_msg = Int32()
        longitud_msg = Int32()

        temperatura_msg.data = self.temperatura_objetivo
        temperatura_cama_msg.data = self.temperatura_cama_objetivo
        imprimir_msg.data = self.imprimir_objetivo
        longitud_msg.data = self.longitud_extrusion_objetivo

        self.publisher_temperatura_objetivo.publish(temperatura_msg)
        self.publisher_temperatura_cama_objetivo.publish(temperatura_cama_msg)
        self.publisher_imprimir_objetivo.publish(imprimir_msg)
        self.publisher_longitud_extrusion_objetivo.publish(longitud_msg)


    def temperatura_callback(self, msg):
        self.get_logger().info('Temperatura enviada: %d' % self.temperatura_objetivo)
        self.get_logger().info('Temperatura recibida: %d' % msg.data)

    def temperatura_cama_callback(self, msg):
        self.get_logger().info('Temperatura cama recibida: %d' % msg.data)

    def imprimir_callback(self, msg):
        self.get_logger().info('Imprimir recibido: %d' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    humble_node = HumbleNode()
    rclpy.spin(humble_node)
    humble_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()