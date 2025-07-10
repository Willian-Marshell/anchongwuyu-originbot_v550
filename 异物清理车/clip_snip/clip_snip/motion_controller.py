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
        
        # PID������
        self.angular_pid = PIDController(0.8, 0.01, 0.05, 1.0, -1.0)  # �Ƕȿ���
        self.linear_pid = PIDController(0.5, 0.001, 0.02, 0.5, 0.0)   # �������
        
        self.target_position = None
        self.state = "SEARCHING"  # ״̬��
        self.last_detection_time = time.time()
        
        self.attract_init_time = 0
        self.attract_maintain_time = 0.7 #s
        self.backword_maintain_time = 2 #s
        
        self.channel = 16
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.channel, GPIO.OUT, initial=GPIO.LOW)    #�͵�ƽ��ʼ��
        
        # �������ƶ�ʱ��
        self.create_timer(0.05, self.control_loop)  # 20Hz����Ƶ��

    def clip_callback(self, msg):
        self.target_position = msg
        self.last_detection_time = time.time()

    def control_loop(self):
        if not self.target_position:
            return
            
        twist_msg = Twist()
        magnet_msg = Bool()
        
        # ����ʱ���
        dt = 0.1  # �̶���������
        
        if self.state == "SEARCHING":
            # ��ת����Ŀ��
            twist_msg.angular.z = 0.5
            if self.target_position.z > 0:
                self.state = "ALIGNING"
                
        elif self.state == "ALIGNING":
            # ˮƽλ��PID���� (Ŀ�꣺ͼ������x=0.5)
            angular_vel = self.angular_pid.compute(0.5, self.target_position.x, dt)
            twist_msg.angular.z = angular_vel
            
            # ����׼���С����ֵʱ�л�״̬
            if abs(self.target_position.x - 0.5) < 0.05:
                self.state = "APPROACHING"
                GPIO.output(self.channel, GPIO.HIGH)
                
        elif self.state == "APPROACHING":
            # �Ƕȿ��Ʊ��ֶ�׼
            angular_vel = self.angular_pid.compute(0.5, self.target_position.x, dt)
            twist_msg.angular.z = angular_vel
            
            # ���ڼ��Ӵ�С�ľ������ (Ŀ���С��ֵ)
            #distance_error = 10000 - self.target_position.z  # ʾ����ֵ
            #linear_vel = self.linear_pid.compute(0, distance_error, dt)
            #twist_msg.linear.x = linear_vel
            twist_msg.linear.x = 0.15
            
            # �������㹻��ʱֹͣ
            if self.target_position.y >0.85:  # ����ʵ�ʵ���
                twist_msg.linear.x = 0.0
                magnet_msg.data = True
                self.state = "ATTRACT"      #��һ��������
                
                self.attract_init_time = time.time()
        elif self.state == "ATTRACT":
            if (time.time() - self.attract_init_time) < self.attract_maintain_time:    #ά��0.1m/s�ٶ�һ��ʱ��
                twist_msg.linear.x = 0.1
            else:
                twist_msg.linear.x = 0.0
                self.state = "BACKWORD"    #��һ��������
                self.attract_init_time = time.time()
        elif self.state == "BACKWORD":
            if (time.time() - self.attract_init_time) < self.backword_maintain_time:    #ά��0.1m/s�ٶ�һ��ʱ��
                twist_msg.linear.x = -0.3
            else:
                twist_msg.linear.x = 0.0
                self.state = "COMPLETED"    #����
                self.attract_init_time = time.time()
        
        elif self.state == "COMPLETED":
            GPIO.output(self.channel, GPIO.LOW)
            if (time.time() - self.attract_init_time) < self.backword_maintain_time:    #ά��0.1m/s�ٶ�һ��ʱ��
                twist_msg.linear.x = -0.1
            magnet_msg.data = False
            
        # ������������
        self.cmd_vel_pub.publish(twist_msg)
        self.magnet_pub.publish(magnet_msg)
        
        # ��ʱ����
        if time.time() - self.last_detection_time > 2.0:  # 2��δ��⵽Ŀ��
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
