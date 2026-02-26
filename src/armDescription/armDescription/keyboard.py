import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, tty, termios, threading

NUM_JOINTS = 3  # change to match your arm
SPEED = 0.5

class KeyVelController(Node):
    def __init__(self):
        super().__init__('key_vel_controller')
        self.pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.velocities = [0.0] * NUM_JOINTS
        self.timer = self.create_timer(0.05, self.publish)
        print("Keys: 1/q=j1, 2/w=j2 ... space=stop")

    def publish(self):
        msg = Float64MultiArray()
        msg.data = self.velocities
        self.pub.publish(msg)

    def set_vel(self, idx, val):
        self.velocities = [0.0] * NUM_JOINTS
        self.velocities[idx] = val

    def stop(self):
        self.velocities = [0.0] * NUM_JOINTS

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = KeyVelController()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    key_map = {
        '1': (0,  SPEED), 'q': (0, -SPEED),
        '2': (1,  SPEED), 'w': (1, -SPEED),
        '3': (2,  SPEED), 'e': (2, -SPEED),
    }

    while True:
        k = get_key()
        if k == ' ':
            node.stop()
        elif k == '\x03':  # Ctrl+C
            break
        elif k in key_map:
            idx, vel = key_map[k]
            node.set_vel(idx, vel)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()