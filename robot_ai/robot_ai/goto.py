import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
import numpy as np
import time
from robot_ai.amcl import AMCL
# from robcomp_util.odom import Odom

class GoTo(Node, AMCL): # Mude o nome da classe
    def __init__(self,):
        Node.__init__(self, 'goto_node') # Mude o nome do nó
        AMCL.__init__(self) # Mude o nome do nó
        self.timer = None

        # Inicialização de variáveis
        self.twist = Twist()
        self.threshold = np.pi/180
        self.kp_linear = 0.6
        self.kp_angular = 0.6
        self.max_vel = 0.2
        self.index = 0

        self.robot_state = 'done'
        self.state_machine = {
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop,
            'done': self.stop
        }

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)


    def reset(self, point = Point(x=0., y=0., z=0.)):
        self.twist = Twist()
        self.robot_state = 'center' # Inicie a ação
        if self.timer is None:
            self.timer = self.create_timer(0.25, self.control) # Timer para o controle
        
        ### Iniciar variaveis da ação
        self.point = point

    def get_angular_error(self):
        x = self.point.x - self.x
        y = self.point.y - self.y
        theta = np.arctan2(y , x)

        self.distance = np.sqrt(x**2 + y**2)
        erro = theta - self.yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

        print('Erro: ', self.erro)
        self.twist.angular.z = self.erro * self.kp_angular

    def center(self):
        self.get_angular_error()

        if abs(self.erro) < np.deg2rad(3):
            self.robot_state = 'goto'

    def goto(self):
        self.get_angular_error()

        if self.distance > 0.01:
            linear_x = self.distance * self.kp_linear
            self.twist.linear.x = min(linear_x, self.max_vel)

        else:
            self.robot_state = 'stop'
    
    def stop(self):
        self.twist = Twist() # Zera a velocidade
        print("Parando o robô.")
        self.timer.cancel() # Finaliza o timer
        self.timer = None # Reseta a variável do timer
        self.robot_state = 'done' # Ação finalizada


    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args) # Inicia o ROS2
    ros_node = GoTo() # Cria o nó

    rclpy.spin_once(ros_node) # Processa as callbacks uma vez
    point = Point()
    point.x = 1.0
    point.y = 0.0
    ros_node.reset(point) # Reseta o nó para iniciar a ação

    while not ros_node.robot_state == 'done': # Enquanto a ação não estiver finalizada
        rclpy.spin_once(ros_node) # Processa os callbacks e o timer

    ros_node.destroy_node() # Destroi o nó
    rclpy.shutdown()    # Finaliza o ROS2

if __name__ == '__main__': # Executa apenas se for o arquivo principal
    main()