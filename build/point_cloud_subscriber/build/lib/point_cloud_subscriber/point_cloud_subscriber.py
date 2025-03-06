#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple
import sys
import serial
import termios
import tty
from threading import Thread
import time
import rclpy
import math
# 配置参数
LW = 0.2
RW = -0.2
WF = -0.5

class PointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('point_cloud_visualizer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud',
            self.cloud_callback,
            10)
        
        # 初始化Matplotlib
        plt.ion()  # 开启交互模式
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Point Cloud Visualization')
        plt.grid(True)
        
        # 绘制基准线
        self.baseline, = self.ax.plot([], [], 'g--', linewidth=1)
        # 初始化动态元素
        self.left_scatter = self.ax.scatter([], [], c='blue', s=10, label='Left Points')
        self.right_scatter = self.ax.scatter([], [], c='red', s=10, label='Right Points')
        self.left_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Left Boundary')
        self.right_line, = self.ax.plot([], [], 'r-', linewidth=2, label='Right Boundary')
        plt.legend()
        
        self.get_logger().info('\033[1;32m----> Visualizer Started. Opening display window...\033[0m')

        self.ser = serial.Serial(
            port="/dev/ttyTHS0",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self.get_logger().info(f"串口已连接: {self.ser.name}")
        
        # 初始化终端设置
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # 控制参数初始化
        self.direction = 2500
        self.speed = 780
        self.last_key_time = time.time()

        # 启动键盘监听线程
        self.is_running = True
        self.key_thread = Thread(target=self.read_keyboard)
        self.key_thread.daemon = True
        self.key_thread.start()

        # 创建定时器（每0.1秒发送一次指令）
        self.timer = self.create_timer(0.1, self.send_commands)
        self.get_logger().info("等待 WASD 输入（Ctrl+C 退出）...")

    def read_keyboard(self):
        """键盘监听线程"""
        try:
            while self.is_running and rclpy.ok():
                key = sys.stdin.read(1)
                self.process_key(key)
        except Exception as e:
            self.get_logger().error(f"键盘错误: {str(e)}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

    def process_key(self, key):
        """处理按键事件"""
        if key == 'w':
            self.speed = 810
            self.get_logger().info("前进", throttle_duration_sec=1)
        elif key == 's':
            self.speed = 750
            self.get_logger().info("后退", throttle_duration_sec=1)
        elif key == 'a':
            self.direction = 2000
            self.get_logger().info("左转", throttle_duration_sec=1)
        elif key == 'd':
            self.direction = 3000
            self.get_logger().info("右转", throttle_duration_sec=1)
        elif key == 'e':
            self.speed = 780
            self.get_logger().info("右转", throttle_duration_sec=1)
        elif key == 'f':
            self.speed = 780
            self.get_logger().info("右转", throttle_duration_sec=1)
        elif key == ' ':
            if self.speed == 810:
                self.direction, self.speed = 2500, 500  
            elif self.speed == 720:
                self.direction, self.speed = 2500, 1000
            else:
                self.direction, self.speed = 2500, 780        
            self.get_logger().info("急停", throttle_duration_sec=1)
        elif key == '\x03':  # Ctrl+C
            self.get_logger().info("退出指令接收")
            self.stop()
            rclpy.try_shutdown()
            return
        self.last_key_time = time.time()

    def send_commands(self):
        """定时发送指令"""
        # 0.3秒无操作自动归零
        if time.time() - self.last_key_time > 0.5:
            if self.speed == 810 or  self.speed == 750:
                self.get_logger().info("自动停车", throttle_duration_sec=1)
            self.speed = 780
            
        # 构造协议指令
        cmd = f"#P1={self.direction:.2f}!\n#P2={self.speed}!\n"
        # print(self.direction)
        try:
            if self.ser.is_open:
                self.ser.write(cmd.encode())
                self.get_logger().debug(f"发送: {cmd.strip()}")
        except Exception as e:
            self.get_logger().error(f"串口错误: {str(e)}")
            self.stop()

    def stop(self):
        """安全关闭资源"""
        if self.is_running:
            self.is_running = False
            self.ser.close()
            # 显式恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
            self.get_logger().info("串口连接已关闭")
            
    def cloud_callback(self, msg):
        Point2D = namedtuple('Point2D', ['x', 'y', 'ang'])
        left_list = []
        right_list = []

        # 处理点云数据
        for data in pc2.read_points(msg, field_names=('x', 'y', 'z')):
            x, y, z = data
            if x > 0 and y > 0:
                ang = 1000 * (y - LW) / (x - WF)
                left_list.append(Point2D(x, y, ang))
            elif x > 0 and y < 0:
                ang = 1000 * (y - RW) / (x - WF)
                right_list.append(Point2D(x, y, ang))

        # 排序和角度计算（保持原有逻辑）
        left_list.sort(key=lambda p: p.x)
        right_list.sort(key=lambda p: p.x)
        
        AngleLeft = Point2D(0, 0, sys.maxsize)
        AngleRight = Point2D(0, 0, -sys.maxsize-1)
        # AngleLeftLast = Point2D(0, 0, left_list[0].ang)
        # AngleRightLast = Point2D(0, 0, right_list[0].ang)

        for p in left_list:
            if p.ang < AngleLeft.ang and p.x < 2:
                AngleLeft = p
            # AngleLeftLast = AngleLeft

        for p in right_list:
            if p.ang > AngleRight.ang and p.x < 2 :
                AngleRight = p
            # AngleRightLast = AngleRight
        self.direction = - math.atan(AngleLeft.ang / 1000.0 ) * 1000 - math.atan(AngleRight.ang / 1000.0 ) * 1000 + 2500
        if self.direction > 3000:
            self.direction = 3000
        if self.direction < 2000:
            self.direction = 2000
        
        # print(AngleLeft,AngleRight)
        # 更新可视化
        self.update_visualization(left_list, right_list, LW, RW, WF, AngleLeft, AngleRight)

    def update_visualization(self, left, right, lw, rw, wf, angle_left, angle_right):
        # 清空旧数据但保留背景元素
        try:
            self.left_scatter.remove()
            self.right_scatter.remove()
            self.base_points.remove()
            self.boundary_points.remove()
        except:
            pass

        # 绘制新数据
        if left:
            self.left_scatter = self.ax.scatter(
                [p.x for p in left],
                [p.y for p in left],
                c='blue', s=10, alpha=0.5, label='Left Points')

        if right:
            self.right_scatter = self.ax.scatter(
                [p.x for p in right],
                [p.y for p in right],
                c='red', s=10, alpha=0.5, label='Right Points')

        # 绘制四个关键点（加大尺寸并改变形状）
        base_points = self.ax.scatter(
            [wf, wf],  # LW和RW的X坐标
            [lw, rw],   # WF的Y坐标
            c='green', marker='s', s=100, edgecolors='black', 
            label='Base Points', zorder=5
        )

        boundary_points = self.ax.scatter(
            [angle_left.x, angle_right.x],
            [angle_left.y, angle_right.y],
            c='orange', marker='*', s=200, edgecolors='black',
            label='Boundary Points', zorder=5
        )

        # 绘制边界线（加粗线宽）
        self.left_line.set_data([wf, angle_left.x], [lw, angle_left.y])
        self.left_line.set(linewidth=3, linestyle='-', color='blue')
    
        self.right_line.set_data([wf, angle_right.x], [rw, angle_right.y])
        self.right_line.set(linewidth=3, linestyle='-', color='red')

        # 保存引用防止被垃圾回收
        self.base_points = base_points
        self.boundary_points = boundary_points


        # 调整坐标轴范围（增加10%边界裕量）
        all_x = [p.x for p in left + right] + [lw, rw, angle_left.x, angle_right.x]
        all_y = [p.y for p in left + right] + [wf, angle_left.y, angle_right.y]
    
        if all_x and all_y:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            x_pad = (x_max - x_min) * 0.1
            y_pad = (y_max - y_min) * 0.1
        
            self.ax.set_xlim(x_min - x_pad, x_max + x_pad)
            self.ax.set_ylim(y_min - y_pad, y_max + y_pad)

        # 添加图例（调整位置防止遮挡）
        if not hasattr(self, 'legend_added'):
            self.ax.legend(loc='upper right', framealpha=0.8)
            self.legend_added = True

        # 强制刷新显示
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    visualizer = PointCloudVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        plt.close('all')  # 关闭所有Matplotlib窗口
        visualizer.get_logger().info('\033[1;33m----> Visualization window closed.\033[0m')
    finally:
        visualizer.destroy_node()
        visualizer.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()