from rclpy.node import Node
import serial
import sys
import termios
import tty
from threading import Thread
import time
import rclpy

class KeyboardSerialControl(Node):
    def __init__(self):
        super().__init__('keyboard_serial_control')
        # 串口初始化
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
            self.direction, self.speed = 2500, 780
            
        # 构造协议指令
        cmd = f"#P1={self.direction:.2f}!\n#P2={self.speed}!\n"
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

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSerialControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("主进程关闭")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()