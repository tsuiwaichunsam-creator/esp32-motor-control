#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import select
import tty
import termios

USAGE = """
--------------------------------------------------
🎮 JOINT ANGLE KEYBOARD CONTROLLER
--------------------------------------------------
Controls:
  [W] : Increase angle by 10°
  [S] : Decrease angle by 10°
  [D] : Increase angle by 1°
  [A] : Decrease angle by 1°
  
  [Spacebar] : Go to 0° (zero)
  [Q]        : Quit Controller

Current angle limit: ±180° (adjust in script)
--------------------------------------------------
"""

class JointKeyboardNode(Node):
    def __init__(self):
        super().__init__('joint_keyboard_node')
        # 发布目标角度到 ESP32
        self.cmd_pub = self.create_publisher(Float64, '/joint/command', 10)
        # 当前目标角度（内部跟踪）
        self.current_angle = 0.0
        # 限位（与 ESP32 一致，可修改）
        self.min_angle = -180.0
        self.max_angle = 180.0
        self.get_logger().info("Joint Keyboard Controller started.")

    def send_angle(self, angle):
        """发布角度并更新内部值"""
        # 限幅
        clamped = max(self.min_angle, min(self.max_angle, angle))
        if clamped != angle:
            print(f"⚠️ Angle {angle:.1f}° clamped to {clamped:.1f}° (limit ±{self.max_angle}°)")
        self.current_angle = clamped
        msg = Float64()
        msg.data = clamped
        self.cmd_pub.publish(msg)
        print(f"📡 Sent: {clamped:.1f}°")

    def step(self, delta):
        """步进改变角度"""
        new_angle = self.current_angle + delta
        self.send_angle(new_angle)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = JointKeyboardNode()
    settings = termios.tcgetattr(sys.stdin)

    print(USAGE)

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '':
                continue

            key_lower = key.lower()

            if key_lower == 'w':
                node.step(10.0)          # 大步增加
            elif key_lower == 's':
                node.step(-10.0)         # 大步减少
            elif key_lower == 'd':
                node.step(1.0)           # 小步增加
            elif key_lower == 'a':
                node.step(-1.0)          # 小步减少
            elif key == ' ':
                node.send_angle(0.0)     # 归零
            elif key_lower == 'q' or key == '\x03':
                break

    except Exception as e:
        print(e)
    finally:
        # 退出前可发送停止命令（归零）或保持最后位置
        node.send_angle(node.current_angle)  # 保持当前角度
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

