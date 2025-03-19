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
from matplotlib.colors import ListedColormap
from sklearn.neighbors import KDTree
import numpy as np
from typing import List, Tuple
from collections import defaultdict
from matplotlib.patches import Wedge 
# 配置参数
LW = 0.3
RW = -0.3
WF = -0.1
class Point2D:
    def __init__(self, x: float, y: float, ang: float, kind: int):
        self.x = x
        self.y = y
        self.ang = ang
        self.kind = kind
    
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
        self.scatters = {}
        self.cmap = ListedColormap(['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'])
        self.boundary_artists = []  # 存储边界相关图形对象
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
        self.direction = 2700
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

        self.min_ang = -math.pi/2  # 角度范围下限
        self.max_ang = math.pi/2  # 角度范围上限
        self.kind_ranges = defaultdict(lambda: (math.inf, -math.inf))  # 存储各kind的角度范围
        self.merged_ranges = []  # 合并后的区间
        self.max_gap = (0.0, 0.0)  # 最大空隙（起始角度，结束角度）

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
    
    # def dfs(self, points):
    #     range_thre = 1
    #     stack = []
    #     kind = 0
    #     for idx in range(len(points)):
    #         if points[idx].kind == 0:
    #             stack.append(idx)
    #             kind += 1
    #             while len(stack) > 0:
    #                 front = stack[-1]
    #                 stack.pop()
    #                 for i in range(1,6):
    #                     if front+i < len(points) and points[front+i].kind == 0 :
    #                         dis = math.sqrt((points[front].x - points[front+i].x) * (points[front].x - points[front+i].x) + (points[front].y - points[front+i].y) * (points[front].y - points[front+i].y))
    #                         if   dis < range_thre:
    #                             points[front+i].kind = kind
    #                             stack.append(front+i)

    #     print(kind)
    #     return points
    
    def cluster_points(self, points, radius=1.0):
        # 构建KDTree
        coords = np.array([[p.x, p.y] for p in points])
        tree = KDTree(coords)
        cluster_id = 0
        cluster_sizes = {}  # 用于记录每个聚类的大小
        for i in range(len(points)):
            if points[i].kind == 0:
                cluster_id += 1
                stack = [i]
                cluster_size = 0  # 记录当前聚类的大小
                while stack:
                    idx = stack.pop()
                    if points[idx].kind == 0:
                        points[idx].kind = cluster_id
                        cluster_size += 1  # 增加当前聚类的点数
                        # 查找半径内所有点
                        neighbors = tree.query_radius(
                        [coords[idx]], 
                        r=radius
                        )[0]
                        for n in neighbors:
                            if points[n].kind == 0:
                                stack.append(n)
            cluster_sizes[cluster_id] = cluster_size
        for i in range(len(points)):
            if points[i].kind in cluster_sizes and cluster_sizes[points[i].kind] < 10:
                points[i].kind = 0  # 重置为未被聚类的标记
        # print(f"Total clusters: {cluster_id}")
        return points

    def process_key(self, key):
        """处理按键事件"""
        if key == 'w':
            self.speed = 810
            self.get_logger().info("前进", throttle_duration_sec=1)
        elif key == 's':
            self.speed = 750
            self.get_logger().info("后退", throttle_duration_sec=1)
        elif key == 'a':
            self.direction = 2200
            self.get_logger().info("左转", throttle_duration_sec=1)
        elif key == 'd':
            self.direction = 3200
            self.get_logger().info("右转", throttle_duration_sec=1)
        elif key == 'e':
            self.speed = 780
            self.get_logger().info("右转", throttle_duration_sec=1)
        elif key == 'f':
            self.speed = 780
            self.get_logger().info("右转", throttle_duration_sec=1)
        elif key == ' ':
            if self.speed == 810:
                self.direction, self.speed = 2700, 500  
            elif self.speed == 720:
                self.direction, self.speed = 2700, 1000
            else:
                self.direction, self.speed = 2700, 780        
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

    def visualization(self, points):

        """更新可视化元素"""
        # 清除旧数据
        for artist in self.boundary_artists:
            artist.remove()
        self.boundary_artists = []
    
        # 获取所有唯一kind值
        unique_kinds = list(set(p.kind for p in points))
        # print(unique_kinds)
        # 更新各分类点云
        count = -1

        if hasattr(self, 'max_gap') and self.max_gap is not None:
            # 将极坐标转换为笛卡尔坐标边界
            start_ang, end_ang = self.max_gap
            x_center, y_center = 0, 0  # 假设原点在中心
            radius = max(self.ax.get_xlim()[1], self.ax.get_ylim()[1]) * 1.5
        
            # 创建扇形区域
            wedge = Wedge(
            center=(x_center, y_center),
            r=radius,
            theta1=np.degrees(start_ang),
            theta2=np.degrees(end_ang),
            color='limegreen',
            alpha=0.2,
            zorder=0  # 确保在点云下方
            )
            self.ax.add_patch(wedge)
            self.boundary_artists.append(wedge)

        for kind in unique_kinds:  # ✅ 遍历实际存在的kind值
            count += 1
            # 筛选当前kind的点
            # if kind == 0:
            #     continue
            filtered_points = [p for p in points if p.kind == kind]
            x = [p.x for p in filtered_points]
            y = [p.y for p in filtered_points]
        
            # 计算颜色索引（避免超过色表长度）
            color_idx = count % self.cmap.N
        
            if count in self.scatters:
                # 更新现有散点图
                self.scatters[count].set_offsets(np.column_stack((x, y)))
                self.scatters[count].set_color(self.cmap(color_idx))
            else:
                # 创建新散点图
                scatter = self.ax.scatter(
                    x, y, 
                    c=[self.cmap(color_idx)], 
                    s=20, 
                    alpha=0.7,
                    edgecolor='w',
                    linewidth=0.3,
                    label=f'Kind {count}'
                )
                self.scatters[count] = scatter
    
        # 更新图例（仅在有新类别时）
        if unique_kinds:
            handles, labels = self.ax.get_legend_handles_labels()
            if len(handles) > len(unique_kinds):
                self.ax.legend(handles[:len(unique_kinds)], 
                              labels[:len(unique_kinds)], 
                              loc='upper right')

        # 自动调整坐标轴
        all_x = [p.x for p in points]
        all_y = [p.y for p in points]
        if all_x and all_y:
            self.ax.set_xlim(min(all_x)-1, max(all_x)+1)
            self.ax.set_ylim(min(all_y)-1, max(all_y)+1)

        # 刷新显示
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def process_points(self, points: List) -> None:
        """扫描所有点，记录每个kind的角度范围"""
        for p in points:
            if p.kind != 0:
                self._update_range(p.kind, p.ang)

    def _update_range(self, kind: int, ang: float) -> None:
        """更新指定kind的角度范围"""
        current_min, current_max = self.kind_ranges[kind]
        self.kind_ranges[kind] = (
            min(current_min, ang),
            max(current_max, ang)
        )

    def merge_ranges(self) -> None:
        """合并所有kind的角度范围"""
        # 收集所有区间并排序
        all_ranges = []
        for min_val, max_val in self.kind_ranges.values():
            if min_val != math.inf:  # 过滤未更新的默认值
                all_ranges.append((min_val, max_val))
        # print(all_ranges)
        # 按起始角度排序
        all_ranges.sort(key=lambda x: x[0])
        
        # 合并重叠区间
        if not all_ranges:
            return
        # print(all_ranges)
        merged = [all_ranges[0]]
        for current_min, current_max in all_ranges[1:]:
            last_min, last_max = merged[-1]
            if current_min <= last_max:
                merged[-1] = (min(last_min, current_min), 
                             max(last_max, current_max))
            else:
                merged.append((current_min, current_max))
        self.merged_ranges = merged
        # print(merged)

    def find_max_gap(self) -> Tuple[float, float]:
        """找到最大空隙区域"""
        gaps = []

        # 检查左边界空隙
        first_min, first_max = self.merged_ranges[0]
        if first_min > self.min_ang:
            gaps.append((self.min_ang, first_min))

        # 检查中间空隙
        for i in range(1, len(self.merged_ranges)):
            prev_max = self.merged_ranges[i-1][1]
            curr_min = self.merged_ranges[i][0]
            if curr_min > prev_max:
                gaps.append((prev_max, curr_min))

        # 检查右边界空隙
        last_min, last_max = self.merged_ranges[-1]
        if last_max < self.max_ang:
            gaps.append((last_max, self.max_ang))

        # 找到最大空隙
        if gaps:
            max_gap = max(gaps, key=lambda x: x[1]-x[0])
        else:
            max_gap = (0.0, 0.0)  # 无空隙
            
        self.max_gap = max_gap
        return max_gap
    
    def find_max_distance_points(self, filtered, angles):
        """根据角度降序排列，寻找相邻点最大间距"""
        # 参数校验
        if len(filtered) < 2:
            return None, None, 0.0
        
        # 按角度从大到小排序（新增排序方向控制）
        # sort_idx = np.argsort(angles)  # 关键修改：负号实现降序
        # sorted_points = filtered[sort_idx]
        
        # 计算相邻点欧氏距离（向量化计算）
        dx = np.diff(filtered[:, 0])  # x坐标差值
        dy = np.diff(filtered[:, 1])  # y坐标差值
        distances = np.sqrt(dx**2 + dy**2)  # 欧氏距离
        
        # 寻找最大间距
        if len(distances) == 0:
            return None, None, 0.0
        
        max_idx = np.argmax(distances)
        print(distances[max_idx])
        # 返回对应点对及距离
        return (angles[max_idx], angles[max_idx+1])

    def cloud_callback(self, msg):
        """优化后的点云处理流程"""
        # 使用numpy向量化读取点云数据
        points = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'))))
        
        # 第一步：快速过滤和角度计算
        mask = (points[:,0] > 0) & (np.linalg.norm(points[:,:2], axis=1) < 5)
        filtered = points[mask]
        
        # 向量化计算角度
        angles = np.arctan2(filtered[:,1], filtered[:,0])
        
        # 第二步：排序优化
        sort_idx = np.argsort(angles)
        sorted_angles = angles[sort_idx]
        filtered = filtered[sort_idx]
        # 寻找最大间隔（向量化实现）
        angle_diff = np.diff(sorted_angles)
        if len(angle_diff) > 0:
            max_gap_idx = np.argmax(angle_diff)
            max_gap = (sorted_angles[max_gap_idx], sorted_angles[max_gap_idx+1])
                # 获取最大间隔对应的两个点
            gap_point1 = filtered[sort_idx][max_gap_idx]     # 前点坐标 (x,y,z)
            gap_point2 = filtered[sort_idx][max_gap_idx + 1]  # 后点坐标 (x,y,z)
            
            # 计算欧氏距离（仅使用x,y坐标）
            gap_distance = np.sqrt(
                (gap_point2[0] - gap_point1[0])**2 +  # x轴差值平方
                (gap_point2[1] - gap_point1[1])**2    # y轴差值平方
            )
        else:
            max_gap = (-math.pi/2, math.pi/2)
            gap_distance = 0.0
        if gap_distance < 0.8:
            # print(gap_distance)
            max_gap = self.find_max_distance_points(filtered, sorted_angles)
            # max_gap = (angles[mad_idx], angles[mad_idx+1])
        
        # 第三步：分割左右区域（布尔索引加速）
        ang_mid = np.mean(max_gap)
        left_mask = sorted_angles > ang_mid
        right_mask = ~left_mask
        
        # 第四步：极值计算优化
        left_points = filtered[sort_idx][left_mask]
        right_points = filtered[sort_idx][right_mask]
        
        # 左区域角度计算（向量化）
        if len(left_points) > 0:
            left_ang = 1000 * (left_points[:,1] - LW) / (left_points[:,0] - WF)
            min_left = np.argmin(left_ang)
            AngleLeft = Point2D(left_points[min_left,0], 
                                left_points[min_left,1], 
                                left_ang[min_left], 0)
        else:
            AngleLeft = 0
        
        # 右区域角度计算（向量化）
        if len(right_points) > 0:
            right_ang = 1000 * (right_points[:,1] - RW) / (right_points[:,0] - WF)
            max_right = np.argmax(right_ang)
            AngleRight = Point2D(right_points[max_right,0], 
                                right_points[max_right,1], 
                                right_ang[max_right], 0)
        else:
            AngleRight = 0
        
        # 最终方向计算
        self.direction = - (math.atan(AngleLeft.ang/1000) + math.atan(AngleRight.ang/1000)) * 500 + 2700
        # self.direction = 2700
        print(self.direction)
        # self.visualization(point_list)
        # print(AngleLeft,AngleRight)
        # 更新可视化
        # print(type(left_points))
        # self.update_visualization(left_points, right_points, LW, RW, WF, AngleLeft, AngleRight)

    def update_visualization(self, left, right, lw, rw, wf, angle_left, angle_right):
        # 清空旧数据但保留背景元素
        try:
            # 移除特定绘图元素
            for artist in [self.left_scatter, self.right_scatter, 
                        self.base_points, self.boundary_points]:
                if artist is not None:
                    artist.remove()
            
            # 清除线条（需要单独处理）
            # self.left_line.remove()
            # self.right_line.remove()
        except (AttributeError, ValueError) as e:
            # 处理部分元素不存在的情况
            print(f"Cleanup warning: {str(e)}")

        # 绘制新数据
        # if left:
        #     self.left_scatter = self.ax.scatter(
        #         [p.x for p in left],
        #         [p.y for p in left],
        #         c='blue', s=10, alpha=0.5, label='Left Points')

        # if right:
        #     self.right_scatter = self.ax.scatter(
        #         [p.x for p in right],
        #         [p.y for p in right],
        #         c='red', s=10, alpha=0.5, label='Right Points')
        print(left.shape)
        if left.any():  # 检查是否存在有效点
            # 绘制散点图（直接使用NumPy数组切片）
            self.left_scatter = self.ax.scatter(
                left[:, 0],  # x坐标列
                left[:, 1],  # y坐标列
                c='blue', 
                s=10, 
                alpha=0.5, 
                label='Left Points'
            )

        if right.any():  # 检查是否存在有效点
            # 绘制散点图（直接使用NumPy数组切片）
            self.left_scatter = self.ax.scatter(
                right[:, 0],  # x坐标列
                right[:, 1],  # y坐标列
                c='red', 
                s=10, 
                alpha=0.5, 
                label='Right Points'
            )
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


        # 自动调整坐标范围（带安全边界）
        all_x = np.concatenate([left[:,0], right[:,0], [wf, angle_left.x, angle_right.x]])
        all_y = np.concatenate([left[:,1], right[:,1], [lw, rw, angle_left.y, angle_right.y]])
        
        if all_x.size > 0 and all_y.size > 0:
            x_min, x_max = np.nanmin(all_x), np.nanmax(all_x)
            y_min, y_max = np.nanmin(all_y), np.nanmax(all_y)
            
            # 添加动态边界（5%的扩展量）
            x_padding = max(0.1, (x_max - x_min) * 0.05) 
            y_padding = max(0.1, (y_max - y_min) * 0.05)
            
            self.ax.set_xlim(x_min - x_padding, x_max + x_padding)
            self.ax.set_ylim(y_min - y_padding, y_max + y_padding)

        # 添加图例（调整位置防止遮挡）
        if not hasattr(self, 'legend_added'):
            self.ax.legend(loc='upper right', framealpha=0.8)
            self.legend_added = True

        # 强制刷新显示
        self.fig.canvas.draw_idle()
        self.fig.canvas.start_event_loop(0.001)
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