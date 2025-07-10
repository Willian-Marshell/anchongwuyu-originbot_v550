import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
from .pid_controller import PIDController
import math
import time
import Hobot.GPIO as GPIO

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.subscription = self.create_subscription(
            Point, 'clip_pos', self.clip_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.magnet_pub = self.create_publisher(Bool, 'magnet_control', 10)
        
        # PID控制器
        self.angular_pid = PIDController(0.8, 0.01, 0.05, 1.0, -1.0)  # 角度控制
        self.linear_pid = PIDController(0.5, 0.001, 0.02, 0.5, 0.0)   # 距离控制
        
        self.target_position = None
        self.state = "SEARCHING"  # 状态机
        self.last_detection_time = time.time()
        
        self.attract_init_time = 0
        self.attract_maintain_time = 0.7 #s
        self.backword_maintain_time = 2 #s
        
        self.channel = 16
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.channel, GPIO.OUT, initial=GPIO.LOW)    #低电平初始化
        
        # 创建控制定时器
        self.create_timer(0.05, self.control_loop)  # 20Hz控制频率

    def clip_callback(self, msg):
        self.target_position = msg
        self.last_detection_time = time.time()

    def control_loop(self):
        if not self.target_position:
            return
            
        twist_msg = Twist()
        magnet_msg = Bool()
        
        # 计算时间差
        dt = 0.1  # 固定控制周期
        
        if self.state == "SEARCHING":
            # 旋转搜索目标
            twist_msg.angular.z = 0.5
            if self.target_position.z > 0:
                self.state = "ALIGNING"
                
        elif self.state == "ALIGNING":
            # 水平位置PID控制 (目标：图像中心x=0.5)
            angular_vel = self.angular_pid.compute(0.5, self.target_position.x, dt)
            twist_msg.angular.z = angular_vel
            
            # 当对准误差小于阈值时切换状态
            if abs(self.target_position.x - 0.5) < 0.05:
                self.state = "APPROACHING"
                GPIO.output(self.channel, GPIO.HIGH)
                
        elif self.state == "APPROACHING":
            # 角度控制保持对准
            angular_vel = self.angular_pid.compute(0.5, self.target_position.x, dt)
            twist_msg.angular.z = angular_vel
            
            # 基于夹子大小的距离估计 (目标大小阈值)
            #distance_error = 10000 - self.target_position.z  # 示例阈值
            #linear_vel = self.linear_pid.compute(0, distance_error, dt)
            #twist_msg.linear.x = linear_vel
            twist_msg.linear.x = 0.15
            
            # 当夹子足够大时停止
            if self.target_position.y >0.85:  # 根据实际调整
                twist_msg.linear.x = 0.0
                magnet_msg.data = True
                self.state = "ATTRACT"      #下一步：磁吸
                
                self.attract_init_time = time.time()
        elif self.state == "ATTRACT":
            if (time.time() - self.attract_init_time) < self.attract_maintain_time:    #维持0.1m/s速度一段时间
                twist_msg.linear.x = 0.1
            else:
                twist_msg.linear.x = 0.0
                self.state = "BACKWORD"    #下一步，后退
                self.attract_init_time = time.time()
        elif self.state == "BACKWORD":
            if (time.time() - self.attract_init_time) < self.backword_maintain_time:    #维持0.1m/s速度一段时间
                twist_msg.linear.x = -0.3
            else:
                twist_msg.linear.x = 0.0
                self.state = "COMPLETED"    #结束
                self.attract_init_time = time.time()
        
        elif self.state == "COMPLETED":
            GPIO.output(self.channel, GPIO.LOW)
            if (time.time() - self.attract_init_time) < self.backword_maintain_time:    #维持0.1m/s速度一段时间
                twist_msg.linear.x = -0.1
            magnet_msg.data = False
            
        # 发布控制命令
        self.cmd_vel_pub.publish(twist_msg)
        self.magnet_pub.publish(magnet_msg)
        
        # 超时处理
        if time.time() - self.last_detection_time > 2.0:  # 2秒未检测到目标
            self.state = "SEARCHING"
            self.target_position = None
def main(args=None):
    rclpy.init(args=args)
    Motion_controller = MotionController()
    rclpy.spin(Motion_controller)
    
    GPIO.cleanup()
    Motion_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()            
