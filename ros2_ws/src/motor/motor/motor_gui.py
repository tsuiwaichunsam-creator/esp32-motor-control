#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tkinter as tk
from tkinter import messagebox
from threading import Thread

class MotorGuiNode(Node):
    def __init__(self):
        super().__init__('motor_gui_node')
        # 发布目标角度（度）到 ESP32
        self.cmd_pub = self.create_publisher(Float64, '/joint/command', 10)
        # 订阅当前角度反馈
        self.state_sub = self.create_subscription(Float64, '/joint/state', self.state_callback, 10)
        self.current_angle = 0.0
        self.get_logger().info("Motor GUI Node started.")

    def state_callback(self, msg):
        self.current_angle = msg.data
        # 更新GUI显示（通过回调在主线程中更新，需要线程安全）
        if hasattr(self, 'angle_label') and self.angle_label:
            self.angle_label.config(text=f"{self.current_angle:.2f}°")

    def send_target_angle(self, angle_deg):
        """发布目标角度（度）"""
        msg = Float64()
        msg.data = float(angle_deg)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent target angle: {angle_deg:.2f}°")

class MotorGUIApp:
    def __init__(self, root, node):
        self.node = node
        self.root = root

        root.title("Joint Angle Controller")
        root.geometry("350x300")
        root.configure(bg="#2c3e50")

        # 标题
        tk.Label(root, text="JOINT ANGLE CONTROL", font=("Arial", 12, "bold"),
                 fg="#ecf0f1", bg="#2c3e50").pack(pady=10)

        # ---- 当前角度显示 ----
        frame_state = tk.LabelFrame(root, text=" Current Angle ", fg="#f1c40f",
                                    bg="#2c3e50", font=("Arial", 10, "bold"))
        frame_state.pack(pady=10, fill="x", padx=15)

        # 标签引用，便于更新
        self.angle_label = tk.Label(frame_state, text="0.00°", font=("Arial", 16),
                                    fg="white", bg="#2c3e50")
        self.angle_label.pack(pady=5)
        # 将标签引用存入node，供回调更新
        node.angle_label = self.angle_label

        # ---- 目标角度输入 ----
        frame_cmd = tk.LabelFrame(root, text=" Set Target Angle ", fg="#2ecc71",
                                  bg="#2c3e50", font=("Arial", 10, "bold"))
        frame_cmd.pack(pady=10, fill="x", padx=15)

        tk.Label(frame_cmd, text="Target (deg):", fg="white", bg="#2c3e50").grid(row=0, column=0, sticky="w", pady=5)
        self.target_entry = tk.Entry(frame_cmd, width=10)
        self.target_entry.insert(0, "0.0")
        self.target_entry.grid(row=0, column=1, padx=5, pady=5)

        # 发送按钮
        btn_send = tk.Button(frame_cmd, text="MOVE", width=10, height=1,
                             command=self.send_target)
        btn_send.grid(row=0, column=2, padx=10)

        # ---- 归零/停止 ----
        btn_zero = tk.Button(root, text="GO TO ZERO", bg="#e67e22", fg="white",
                             font=("Arial", 11, "bold"), height=1,
                             command=lambda: self.node.send_target_angle(0.0))
        btn_zero.pack(fill="x", padx=15, pady=5)

        # 退出按钮
        btn_quit = tk.Button(root, text="QUIT", bg="#e74c3c", fg="white",
                             font=("Arial", 11, "bold"), height=1,
                             command=root.quit)
        btn_quit.pack(fill="x", padx=15, pady=5)

    def send_target(self):
        try:
            angle = float(self.target_entry.get())
            self.node.send_target_angle(angle)
        except ValueError:
            messagebox.showerror("Input Error", "Please enter a valid number.")

def main(args=None):
    rclpy.init(args=args)
    node = MotorGuiNode()

    # 在单独线程中运行 ROS 事件循环
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    app = MotorGUIApp(root, node)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()