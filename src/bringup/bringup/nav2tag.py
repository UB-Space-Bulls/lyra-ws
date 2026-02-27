# navigate_to_tag.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math

class NavigateToTag(Node):
    def __init__(self):
        super().__init__('navigate_to_tag')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_sent = False
        self.target_tag = 'tag36h11:0'  # change to your tag ID
        self.stop_distance = 0.5        # stop 0.5m in front of tag

        self.timer = self.create_timer(0.5, self.try_navigate)
        self.get_logger().info('Waiting for tag in TF...')

    def try_navigate(self):
        if self.goal_sent:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.target_tag,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f'Tag not in TF yet: {e}', throttle_duration_sec=2.0)
            return

        # Build goal pose in front of tag
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Offset in front of tag
        goal.pose.position.x = tx - self.stop_distance * math.cos(yaw)
        goal.pose.position.y = ty - self.stop_distance * math.sin(yaw)
        goal.pose.position.z = 0.0

        # Face the tag
        facing_yaw = yaw + math.pi
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(facing_yaw / 2.0)
        goal.pose.orientation.w = math.cos(facing_yaw / 2.0)

        self.send_goal(goal)

    def send_goal(self, pose):
        self.get_logger().info('Sending goal to Nav2...')
        self.nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        future.add_done_callback(self.goal_response_cb)
        self.goal_sent = True

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            self.goal_sent = False
            return
        self.get_logger().info('Goal accepted!')
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        self.get_logger().info('Reached the tag!')

    def feedback_cb(self, feedback):
        dist = feedback.feedback.distance_remaining
        self.get_logger().info(f'Distance to tag: {dist:.2f}m', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NavigateToTag())
    rclpy.shutdown()

if __name__ == '__main__':
    main()