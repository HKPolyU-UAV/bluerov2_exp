#!/usr/bin/env python

import cv2
import apriltag

import numpy as np
import tf
import rospy
# import image_transport
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseStamped

class AprilTagPoseEstimator:
    def __init__(self, camera_matrix, dist_coeffs, tag_size, target_id):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.tag_size = tag_size
        self.target_id = target_id
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
        self.tracked_points = []
        self.poses = []
        self.bridge = CvBridge()
        self.pose_pub = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size=10)
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', CompressedImage, self.image_callback)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        display_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.detect_and_plot(cv_image, display_image)

    def detect_and_plot(self, cv_image, display_image):
        cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
        detections = self.detector.detect(cv_image)
        for detection in detections:
            if detection.tag_id == self.target_id:
                self.publish_pose(detection)
                self.display_trajectory(display_image)
                break

    def publish_pose(self, detection):
        img_points = detection.corners
        center_x = np.mean([point[0] for point in img_points])
        center_y = np.mean([point[1] for point in img_points])
        self.tracked_points.append((int(center_x), int(center_y)))

        obj_points = np.array([
            [-self.tag_size / 2, self.tag_size / 2, 0],
            [self.tag_size / 2, self.tag_size / 2, 0],
            [self.tag_size / 2, -self.tag_size / 2, 0],
            [-self.tag_size / 2, -self.tag_size / 2, 0]
        ])
        
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)
        
        if success:
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            quaternion = tf.transformations.quaternion_from_matrix(transform_matrix)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_frame"
            pose_msg.pose.position.x = tvec[0]
            pose_msg.pose.position.y = tvec[1]
            pose_msg.pose.position.z = tvec[2]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            self.pose_pub.publish(pose_msg)

    def display_trajectory(self, image):
        overlay_image = image.copy()
        num_points = len(self.tracked_points)
        
        if num_points < 2:
            return

        for i in range(1, num_points):
            pt1 = self.tracked_points[i - 1]
            pt2 = self.tracked_points[i]
            color = (0, 0, 255)
            cv2.line(image, pt1, pt2, color, 4)

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub.publish(image_msg)

if __name__ == '__main__':
    rospy.init_node('apriltag_pose_estimator')

    camera_matrix = np.array([
        [628.8233032226562, 0, 646.732666015625],
        [0, 628.3672485351562, 364.4508361816406],
        [0, 0, 1]
    ], dtype=np.float32)

    # -0.056777212768793106, 0.06796900182962418, 0.0007022436475381255, 0.0004860123444814235, -0.021817076951265335

    dist_coeffs = np.array([-0.056777212768793106, 0.06796900182962418, 0.0007022436475381255, 0.0004860123444814235], dtype=np.float32)
    tag_size = 0.3
    target_id = 0

    estimator = AprilTagPoseEstimator(camera_matrix, dist_coeffs, tag_size, target_id)
    rospy.spin()