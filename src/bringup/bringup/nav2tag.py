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

        # Increase TF buffer duration to match rtabmap.yaml tf_buffer_duration
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_sent = False
        self.goal_active = False
        self.target_tag = 'tag36h11:0'
        self.stop_distance = 0.5

        self.timer = self.create_timer(1.0, self.try_navigate)  # was 0.5 — give TF more time between checks
        self.get_logger().info('Waiting for tag in TF...')

    def try_navigate(self):
        # If a goal is currently being executed, don't send another
        if self.goal_active:
            return

        try:
            # Use a small time offset in the past instead of Time(0)
            # This ensures we get a real recent transform, not a stale cached one
            lookup_time = self.get_clock().now() - Duration(seconds=0.1)

            transform = self.tf_buffer.lookup_transform(
                'map',
                self.target_tag,
                lookup_time,
                timeout=Duration(seconds=1.0)   # was 0.5 — more headroom on Jetson
            )
        except tf2_ros.LookupException:
            self.get_logger().warn('Tag not in TF tree yet', throttle_duration_sec=2.0)
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'TF extrapolation error: {e}', throttle_duration_sec=2.0)
            return
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f'TF connectivity error: {e}', throttle_duration_sec=2.0)
            return
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
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

        # Offset stop_distance in front of the tag
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

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
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
            self.goal_active = False   # allow retry
            return
        self.get_logger().info('Goal accepted!')
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result()
        status = result.status

        # status 4 = SUCCEEDED, anything else = failed
        if status == 4:
            self.get_logger().info('Successfully reached the tag!')
            self.goal_active = True   # goal done, don't re-navigate
        else:
            self.get_logger().warn(f'Navigation failed with status {status} — will retry')
            self.goal_active = False  # reset so we retry

    def feedback_cb(self, feedback):
        dist = feedback.feedback.distance_remaining
        self.get_logger().info(f'Distance to tag: {dist:.2f}m', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NavigateToTag())
    rclpy.shutdown()

if __name__ == '__main__':
    main()