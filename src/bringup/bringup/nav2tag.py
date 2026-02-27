#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import rclpy.time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math

class NavigateToTag(Node):
    def __init__(self):
        super().__init__('navigate_to_tag')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_active = False
        self.stop_distance = 0.5
        self.tag_prefix = 'tag36h11'   # matches any tag36h11:* frame

        self.timer = self.create_timer(1.0, self.try_navigate)
        self.get_logger().info('Waiting for any tag36h11 tag in TF...')

    def find_tag_in_tf(self):
        """
        Scan all frames in the TF tree and return the first one
        that starts with the tag prefix (e.g. 'tag36h11:2', 'tag36h11:5').
        Returns None if no tag is visible yet.
        """
        try:
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
        except Exception as e:
            self.get_logger().warn(f'Could not query TF frames: {e}', throttle_duration_sec=3.0)
            return None

        for line in frames_yaml.splitlines():
            # Each frame appears as "  frame_name:" at the start of a block
            line = line.strip()
            if line.endswith(':') and line[:-1].startswith(self.tag_prefix):
                return line[:-1]   # strip trailing colon

        return None

    def try_navigate(self):
        if self.goal_active:
            return

        # Check map→odom exists before doing anything else
        try:
            self.tf_buffer.lookup_transform(
                'map', 'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(
                f'map→odom not available — is RTAB-Map running? ({e})',
                throttle_duration_sec=3.0
            )
            return

        # Dynamically find whichever tag is currently visible
        tag_frame = self.find_tag_in_tf()
        if tag_frame is None:
            self.get_logger().warn('No tag36h11 tag visible in TF tree yet', throttle_duration_sec=2.0)
            return

        self.get_logger().info(f'Found tag: {tag_frame}', throttle_duration_sec=2.0)

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                tag_frame,
                rclpy.time.Time(),          # latest available
                timeout=Duration(seconds=1.0)
            )
        except tf2_ros.LookupException:
            self.get_logger().warn(f'{tag_frame} disappeared from TF', throttle_duration_sec=2.0)
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'TF extrapolation: {e}', throttle_duration_sec=2.0)
            return
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f'TF connectivity (broken chain?): {e}', throttle_duration_sec=2.0)
            return
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

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

        # Stop stop_distance in front of the tag
        goal.pose.position.x = tx - self.stop_distance * math.cos(yaw)
        goal.pose.position.y = ty - self.stop_distance * math.sin(yaw)
        goal.pose.position.z = 0.0

        # Face the tag
        facing_yaw = yaw + math.pi
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(facing_yaw / 2.0)
        goal.pose.orientation.w = math.cos(facing_yaw / 2.0)

        self.get_logger().info(
            f'Navigating to {tag_frame} at ({tx:.2f}, {ty:.2f}) → '
            f'goal ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )
        self.send_goal(goal)

    def send_goal(self, pose):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self.goal_active = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb
        )
        future.add_done_callback(self.goal_response_cb)
        self.goal_active = True

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 — will retry')
            self.goal_active = False
            return
        self.get_logger().info('Goal accepted!')
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result()
        status = result.status
        if status == 4:
            self.get_logger().info('Successfully reached the tag!')
            self.goal_active = True   # stay stopped, don't re-navigate
        else:
            self.get_logger().warn(f'Navigation failed with status {status} — retrying')
            self.goal_active = False

    def feedback_cb(self, feedback):
        dist = feedback.feedback.distance_remaining
        self.get_logger().info(f'Distance to tag: {dist:.2f}m', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NavigateToTag())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
