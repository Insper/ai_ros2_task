import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.task import Future
from robot_ai.map import MapProcessor
from geometry_msgs.msg import Quaternion

class WaypointNavigator(Node, MapProcessor):

    def __init__(self):
        super().__init__('navigator_node')
        MapProcessor.__init__(self)
        # self.orientation = Quaternion()
        # self.last_goal = False
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.goal_future = None
        
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'pathfinder'
        self.state_machine = {
            'pathfinder': self.pathfinder,
            'send_goal': self.send_goal,
            'wait_goal': self.wait_goal,
            'stop': self.stop
        }

    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
    
    def stop(self):
        pass
        assert False, 'Robot has reached the goal.'
    
    def pathfinder(self):
        self.path = self.find_path(self.map_array)[1:]
        
        if self.path is None:
            self.get_logger().error('Path not found.')
            self.robot_state = 'stop'
            return
        self.current_waypoint = 0
        self.robot_state = 'send_goal'

    def compute_orientation_from_last_goal(self, last_goal, current_goal):
        """
        Computes the orientation based on the direction from the last goal to the current goal.

        :param last_goal: Tuple of (x, y) for the last goal position.
        :param current_goal: Tuple of (x, y) for the current goal position.
        :return: A Quaternion representing the orientation from the last goal to the current goal.
        """
        # Calculate the direction vector from the last goal to the current goal
        direction_vector = np.array([current_goal[0] - last_goal[0], current_goal[1] - last_goal[1]])

        # Normalize the direction vector
        norm = np.linalg.norm(direction_vector)
        if norm == 0:
            # If the norm is zero, return a neutral orientation
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        direction_vector /= norm

        # Compute the yaw angle from the direction vector (atan2 gives the angle in radians)
        yaw = np.arctan2(direction_vector[1], direction_vector[0])

        # Convert the yaw angle to a quaternion
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)

        # Create and return the Quaternion
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def send_goal(self):
        # Define your waypoint coordinates here
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal = (self.path[self.current_waypoint][0], self.path[self.current_waypoint][1])
        goal_pose.pose.position.x = goal[0]  # Set the x coordinate
        goal_pose.pose.position.y = goal[1]  # Set the y coordinate
        self.orientation = self.compute_orientation_from_last_goal((self.x,self.y), goal)
        
        goal_pose.pose.orientation = self.orientation  # Set the orientation


        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending goal...')
        self.action_client.wait_for_server()

        # Send goal asynchronously and handle the future
        self.goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Set up a done callback to handle the goal result
        self.goal_future.add_done_callback(self.goal_response_callback)

        self.robot_state = 'wait_goal'
    
    def wait_goal(self):
        if self.goal_future.done() and self.goal_future.result().status == 4:
            self.get_logger().info('Goal reached successfully.')
            self.current_waypoint += 1
            if self.current_waypoint == len(self.path):
                print('GOAL_REACHEABLE: ', self.GOAL_REACHEABLE)
                if self.GOAL_REACHEABLE:
                    self.get_logger().info('All waypoints reached successfully.')
                    self.robot_state = 'stop'
                else:
                    self.robot_state = 'pathfinder'
            else:
                self.robot_state = 'send_goal'
        elif self.goal_future.done() and self.goal_future.result().status in [5, 6, 8]:
            self.get_logger().warn('Goal was aborted, canceled, or failed.')
            assert False, 'Goal was aborted, canceled, or failed.'

    def feedback_callback(self, feedback):
        # self.get_logger().info(f'Current progress: {feedback.feedback}')
        pass

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self.goal_result_future = goal_handle.get_result_async()
        self.goal_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        result = future.result().result
        if future.result().status == 4:
            pass
        else:
            self.get_logger().warn('Goal failed with status: {0}'.format(future.result().status))

def main(args=None):
    rclpy.init(args=args)
    ros_node = WaypointNavigator() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
