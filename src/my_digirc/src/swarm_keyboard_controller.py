#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios

msg = """
Control The Swarm!
---------------------------
Moving around:
        w
   a    s    d

q : Increase altitude (Up)
e : Decrease altitude (Down)

Rotation:
j : Rotate Left (Counter-clockwise)
i : Rotate Right (Clockwise)

Orbit:
c : Toggle Revolution (orbit around center)
r : Toggle Rotation (spin about own axis)
x : Reverse revolution direction
z : Reverse rotation direction

Shapes:
v : V-Formation
l : Line-Formation
m : Square Mission

System:
t : Takeoff
o : Offboard Mode
h : Hover in place
k : Land
Space: Land

CTRL-C to quit
"""

moveBindings = {
    'w': 'FORWARD',
    's': 'BACKWARD',
    'a': 'LEFT',
    'd': 'RIGHT',
    'q': 'UP',
    'e': 'DOWN',
    'j': 'ROTATE_LEFT',
    'i': 'ROTATE_RIGHT',
    'x': 'REVERSE_REVOLUTION',
    'z': 'REVERSE_ROTATION',
}

commandBindings = {
    'v': 'V',
    'l': 'LINE',
    'm': 'SQUARE',
    't': 'TAKEOFF',
    'o': 'OFFBOARD',
    'h': 'HOVER',
    'k': 'LAND',
    ' ': 'LAND',
    'c': 'REVOLVE',
    'r': 'ROTATE',
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardController(Node):
    def __init__(self):
        super().__init__('swarm_keyboard_controller')
        self.publisher_ = self.create_publisher(String, '/swarm/command', 10)
        self.get_logger().info('Keyboard Controller Node started.')

    def publish_command(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {cmd_str}')

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardController()

    print(msg)

    try:
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                node.publish_command(moveBindings[key])
            elif key in commandBindings.keys():
                node.publish_command(commandBindings[key])
            elif key == '\x03': # Ctrl+C
                break
            rclpy.spin_once(node, timeout_sec=0.01)
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
