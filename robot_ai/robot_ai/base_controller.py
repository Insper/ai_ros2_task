import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.task import Future

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(0.25, self.control)
        self.goal_future = None

    def control(self):
        if self.goal_future is None or self.goal_future.done():
            self.send_goal()
        elif self.goal_future.done() and self.goal_future.result().status == 4:
            self.get_logger().info('Goal reached successfully.')
        elif self.goal_future.done() and self.goal_future.result().status in [5, 6, 8]:
            self.get_logger().warn('Goal was aborted, canceled, or failed.')

    def send_goal(self):
        # Define your waypoint coordinates here
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 2.3900000447034837  # Set the x coordinate
        goal_pose.pose.position.y = -0.8499999701976777  # Set the y coordinate
        goal_pose.pose.orientation.w = 1.0  # Set the orientation

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending goal...')
        self.action_client.wait_for_server()

        # Send goal asynchronously and handle the future
        self.goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Set up a done callback to handle the goal result
        self.goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Current progress: {feedback.feedback}')

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
            self.get_logger().info('Goal reached successfully.')
            assert False
        else:
            self.get_logger().warn('Goal failed with status: {0}'.format(future.result().status))

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
