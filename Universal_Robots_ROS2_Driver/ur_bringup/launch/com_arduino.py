import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class HumbleNode(Node):
    def __init__(self):
        super().__init__('micro_ros_arduino_node')
        self.publisher_temperatura = self.create_publisher(Int32, 'temperatura_objetivo', 10)
        self.publisher_pasos = self.create_publisher(Int32, 'pasos_objetivo', 10)
        self.subscription_temperatura = self.create_subscription(Int32, 'temperatura', self.temperatura_callback, 10)
        self.subscription_pasos = self.create_subscription(Int32, 'pasos', self.pasos_callback, 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.temp_count = 200
        self.pasos_count = 5

    def timer_callback(self):
        temperatura_msg = Int32()
        pasos_msg = Int32()
        temperatura_msg.data = self.temp_count
        pasos_msg.data = self.pasos_count
        self.publisher_temperatura.publish(temperatura_msg)
        self.publisher_pasos.publish(pasos_msg)
        #self.temp_count += 1
        #self.get_logger().info('Publicando temperatura objetivo: %d' % temperatura_msg.data)

    def temperatura_callback(self, msg):
        self.get_logger().info('Temperatura recibida: %d' % msg.data)

    def pasos_callback(self, msg):
        self.get_logger().info('Pasos recibidos: %d' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    humble_node = HumbleNode()
    rclpy.spin(humble_node)
    humble_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()