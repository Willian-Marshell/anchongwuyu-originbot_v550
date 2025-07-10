import rclpy, cv2, cv_bridge
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from ai_msgs.msg import PerceptionTargets
from collections import deque
from std_msgs.msg import Int32
import time

from hobot_dnn import pyeasy_dnn as dnn
import ctypes

class hbSysMem_t(ctypes.Structure):
    _fields_ = [
        ("phyAddr",ctypes.c_double),
        ("virAddr",ctypes.c_void_p),
        ("memSize",ctypes.c_int)
    ]

class hbDNNQuantiShift_yt(ctypes.Structure):
    _fields_ = [
        ("shiftLen",ctypes.c_int),
        ("shiftData",ctypes.c_char_p)
    ]

class hbDNNQuantiScale_t(ctypes.Structure):
    _fields_ = [
        ("scaleLen",ctypes.c_int),
        ("scaleData",ctypes.POINTER(ctypes.c_float)),
        ("zeroPointLen",ctypes.c_int),
        ("zeroPointData",ctypes.c_char_p)
    ]

class hbDNNTensorShape_t(ctypes.Structure):
    _fields_ = [
        ("dimensionSize",ctypes.c_int * 8),
        ("numDimensions",ctypes.c_int)
    ]

class hbDNNTensorProperties_t(ctypes.Structure):
    _fields_ = [
        ("validShape",hbDNNTensorShape_t),
        ("alignedShape",hbDNNTensorShape_t),
        ("tensorLayout",ctypes.c_int),
        ("tensorType",ctypes.c_int),
        ("shift",hbDNNQuantiShift_yt),
        ("scale",hbDNNQuantiScale_t),
        ("quantiType",ctypes.c_int),
        ("quantizeAxis", ctypes.c_int),
        ("alignedByteSize",ctypes.c_int),
        ("stride",ctypes.c_int * 8)
    ]

class hbDNNTensor_t(ctypes.Structure):
    _fields_ = [
        ("sysMem",hbSysMem_t * 4),
        ("properties",hbDNNTensorProperties_t)
    ]


class FcosPostProcessInfo_t(ctypes.Structure):
    _fields_ = [
        ("height",ctypes.c_int),
        ("width",ctypes.c_int),
        ("ori_height",ctypes.c_int),
        ("ori_width",ctypes.c_int),
        ("score_threshold",ctypes.c_float),
        ("nms_threshold",ctypes.c_float),
        ("nms_top_k",ctypes.c_int),
        ("is_pad_resize",ctypes.c_int)
    ]


libpostprocess = ctypes.CDLL('/usr/lib/libpostprocess.so')

get_Postprocess_result = libpostprocess.FcosPostProcess
get_Postprocess_result.argtypes = [ctypes.POINTER(FcosPostProcessInfo_t)]
get_Postprocess_result.restype = ctypes.c_char_p

class ClipDetector(Node):
    def __init__(self):
        super().__init__('clip_detector')
        self.bridge = cv_bridge.CvBridge()
        self.sub_clip = False                 #图中含有夹子时，标志位置True
        self.clip_queue_ = deque(maxlen=1)
        
        self.clip_x = 0      #最近夹子（面积在图像中最大）的中心点位置
        self.clip_y = 0
        
        self.bucket_sub = self.create_subscription(PerceptionTargets, '/hobot_dnn_detection', self.clip_callback, 10)
        self.clip_pos_pub = self.create_publisher(Point, '/clip_pos', 10)
        
    def clip_callback(self, msg):
        if not msg or not rclpy.ok():
            return
        self.sub_clip = True    
        self.clip_queue_.append(msg)  
        
        #print(f"\n--- This Frame: FPS = {msg.fps} ---")
        temp_area = 0
        for num, target in enumerate(msg.targets):
            roi = target.rois[0] 
#            print(f"Target {num}: Type: {roi.type}, "
#                  f"x_offset={roi.rect.x_offset}, "
#                  f"y_offset={roi.rect.y_offset}, "
#                  f"height={roi.rect.height}, "
#                  f"width={roi.rect.width}, "
#                  f"conf={roi.confidence:.2f}") 
                  
            new_area = roi.rect.width * roi.rect.height
            if roi.confidence >= 0.45:
                if new_area >= temp_area:
                    temp_area = new_area
                    self.clip_x = roi.rect.x_offset + roi.rect.width/2
                    self.clip_y = roi.rect.y_offset + roi.rect.height/2
            
            
        #发布夹子中心点位置话题
        clip_pose = Point()
        clip_pose.x = self.clip_x / 640.0
        clip_pose.y = self.clip_y / 480.0
        clip_pose.z = float(temp_area)          #面积作为距离参考
        self.clip_pos_pub.publish(clip_pose)
        
def main(args=None):
    rclpy.init(args=args)
    Clipdetector = ClipDetector()
    rclpy.spin(Clipdetector)
    
    Clipdetector.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        