import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import math
import time

import Hobot.GPIO as GPIO

class MagnetController(Node):
    def __init__(self):
        super().__init__('magnet_controller')
        
        #����GPIO���̵���BOARD�����ʽ�µ��źŽӿڣ�16
        self.channel = 16
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.channel, GPIO.OUT, initial=GPIO.LOW)    #�͵�ƽ��ʼ��
        
        self.magnet_sub = self.create_subscription(
            Bool, 'magnet_control', self.magnet_callback, 10)
    
    def magnet_callback(self,msg):
        if not msg or not rclpy.ok():
            return
        if msg.data == True:
            GPIO.output(self.channel, GPIO.HIGH)      #��������ͨ��
        elif msg.data == False:
            GPIO.output(self.channel, GPIO.LOW)      #�ر�������

def main(args=None):
    rclpy.init(args=args)
    magnet_controller = MagnetController()
    rclpy.spin(magnet_controller)
    
    GPIO.cleanup()
    magnet_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()            
