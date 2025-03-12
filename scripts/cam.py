#import numpy as np
import cv2
import os
import rospy
from cv_bridge import CvBridge
#https://stackoverflow.com/questions/73400055/ros-melodic-image-from-webcam


from std_msgs.msg import String
from sensor_msgs.msg import Image


class Camera(object):

    def __init__(self):
        #self.height = 1280
        #self.width = 480
        #self.fps = 10

        self.bridge = CvBridge()
        self.img_w = rospy.get_param('img_width', default=1280)
        self.img_h = rospy.get_param('img_height', default=480)
        self.fps = rospy.get_param('frame_rate', default=30)
        self.auto_exp = rospy.get_param('auto_exposure', default=True)
        #https://blog.csdn.net/smartvxworks/article/details/108792343
        self.exp_level = rospy.get_param('exposure_time', default=-6) 
        self.auto_wb = rospy.get_param('auto_white_balance', default=True)
        

        self.pubstr = "/stereo_camera/raw"
        self.rawpub = rospy.Publisher(self.pubstr, Image, queue_size=10)
        self.cap = cv2.VideoCapture(-1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,self.img_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.img_h)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not self.auto_exp:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) # manual mode
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exp_level)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, self.auto_wb) 
        
        output_info_exp = "For detail of the exposure level, please refer to {}".format('https://blog.csdn.net/smartvxworks/article/details/108792343')
        output_info_cam = "img_w={}\timg_h={}\timg_fps={}\tauto_exposure={}\texposure_level={}\tauto_white_balance={}".format(
                        self.img_w,self.img_h,self.fps,
                        self.auto_exp, self.exp_level, self.auto_wb)
        rospy.loginfo('\n\n'+'*'*15+'CAMERA INFO'+'*'*15 + '\n' + output_info_exp + '\n' + output_info_cam + '\n' + '*'*15+'CAMERA INFO'+'*'*15+'\n\n')
        print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        print(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def run(self):
        if not self.cap.isOpened():
            print("Camera is not opened\n")

        ret, cv_image = self.cap.read()
        #print(cv_image.shape)
        #left_frame = cv_frame[0:480, 0:640]
        #right_frame = cv_frame[0:480, 640:1280]

        if not ret:
            print("Detect no Image\n")

        else:
            #export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libtiff.so.5
            self.rawpub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

if __name__ == '__main__':
    rospy.init_node("Publish_camera")
    camera = Camera()
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        camera.run()
        r.sleep()