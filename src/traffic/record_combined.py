#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading

class VideoSaver:
    def __init__(self, webcam1_topic='/webcam1/image_raw', webcam2_topic='/webcam2/image_raw'):
        rospy.init_node("video_saver", anonymous=True)
        self.bridge = CvBridge()
        self.latest_img_webcam1 = None
        self.latest_img_webcam2 = None
        self.lock1 = threading.Lock()
        self.lock2 = threading.Lock()
        self.stop_event = threading.Event()

        # ROS 이미지 토픽 구독자 설정
        self.img_sub_webcam1 = rospy.Subscriber(webcam1_topic, Image, self.img_callback_webcam1, queue_size=1)
        self.img_sub_webcam2 = rospy.Subscriber(webcam2_topic, Image, self.img_callback_webcam2, queue_size=1)

        # Video writer setup
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('/home/macaron/catkin_ws/src/web_cam/output.avi', self.fourcc, 20.0, (1280, 480))

        # Start image processing thread
        self.image_processing_thread = threading.Thread(target=self.process_images)
        self.image_processing_thread.start()

    def img_callback_webcam1(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            with self.lock1:
                self.latest_img_webcam1 = img
        except CvBridgeError as e:
            rospy.logerr(e)

    def img_callback_webcam2(self, img_msg):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            with self.lock2:
                self.latest_img_webcam2 = img
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_images(self):
        while not self.stop_event.is_set():
            latest_img_webcam1 = None
            latest_img_webcam2 = None

            with self.lock1:
                if self.latest_img_webcam1 is not None:
                    latest_img_webcam1 = self.latest_img_webcam1.copy()

            with self.lock2:
                if self.latest_img_webcam2 is not None:
                    latest_img_webcam2 = self.latest_img_webcam2.copy()

            if latest_img_webcam1 is not None and latest_img_webcam2 is not None:
                combined_img = self.combine_images(latest_img_webcam1, latest_img_webcam2)
                self.out.write(combined_img)
                cv2.imshow('Combined Image', combined_img)
                cv2.waitKey(1)
        
        cv2.destroyAllWindows()
        self.out.release()

    def combine_images(self, img1, img2):
        img1 = cv2.resize(img1, (640, 480))
        img2 = cv2.resize(img2, (640, 480))
        combined_img = np.hstack((img1, img2))
        return combined_img

    def cleanup(self):
        self.stop_event.set()
        self.image_processing_thread.join()
        rospy.signal_shutdown("Shutting down")

def main():
    try:
        vs = VideoSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
