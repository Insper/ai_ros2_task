import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
import numpy as np
import time
from robot_ai.amcl import AMCL
# from robcomp_util.odom import Odom

class GoTo(Node, AMCL):
    def __init__(self,):
        Node.__init__(self, 'goto_node')
        AMCL.__init__(self)
        self.timer = None

        # Variable initialization
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
        self.robot_state = 'center'
        if self.timer is None:
            self.timer = self.create_timer(0.25, self.control)
        
        ### Initialize action variables
        self.point = point

    def get_angular_error(self):
        x = self.point.x - self.x
        y = self.point.y - self.y
        theta = np.arctan2(y , x)

        self.distance = np.sqrt(x**2 + y**2)
        erro = theta - self.yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

        print('Error: ', self.erro)
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
        self.twist = Twist()
        print("Stopping the robot.")
        self.timer.cancel()
        self.timer = None
        self.robot_state = 'done'


    def control(self):
        self.twist = Twist()
        print(f'Current state: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = GoTo()

    rclpy.spin_once(ros_node)
    point = Point()
    point.x = 1.0
    point.y = 0.0
    ros_node.reset(point)

    while not ros_node.robot_state == 'done':
        rclpy.spin_once(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
