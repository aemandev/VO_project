#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from orb_feature_tracker import OrbFeatureTracker
from camera_params import CameraParams
from relative_pose_tracker import RelativePoseTracker
from tf import TransformBroadcaster
from rospy import Time 
from geometry_msgs.msg import Pose
import tf


bridge = CvBridge()
camera_params_ = CameraParams()
feature_tracker = OrbFeatureTracker()
pose_tracker = RelativePoseTracker()
pub_pose_estimation = rospy.Publisher('pose_estimation_topic', Pose, queue_size=10)


def compute_inlier_mask(kp1, kp2):
    size = int(kp1[0].size)
    pt1 = []
    pt2 = []

    for i in range(size):
        pt1.append(kp1[i].pt)
        pt2.append(kp2[i].pt)
    
    max_dist_from_epi_line_in_px = 3.0
    confidence_prob = .99
    F, inlier_mask = cv2.findFundamentalMat(pt1, pt2, cv2.FM_RANSAC, max_dist_from_epi_line_in_px, confidence_prob)
    return inlier_mask

def draw_inliers(img1, img2, kp1, kp2, good_matches, M, mask):
        h,w = feature_tracker.img1.shape[:2]
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        feature_tracker.img2 = cv2.polylines(feature_tracker.img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = mask, # draw only inliers
                   flags = 2)

        img3 = cv2.drawMatches(feature_tracker.img1,feature_tracker.kp1,feature_tracker.img2,feature_tracker.kp2,good_matches,None,**draw_params)

        cv2.imshow('matches', img3)

def detect_and_match_features():
        kp1 = feature_tracker.detect_keypoints(feature_tracker.img1)
        kp2 = feature_tracker.detect_keypoints(feature_tracker.img2)
        kp1, des1 = feature_tracker.describe_keypoints(feature_tracker.img1, kp1)
        kp2, des2 = feature_tracker.describe_keypoints(feature_tracker.img2, kp2)
        # Update feature tracker params
        feature_tracker.kp1 = kp1
        feature_tracker.kp2 = kp2
        feature_tracker.descriptors1 = des1
        feature_tracker.descriptors2 = des2
        matches, good_matches = feature_tracker.match_dscriptors(des1, des2)
        return matches, good_matches

def callback(data,feature_tracker):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow('image', cv2_img)
    cv2.waitKey(1)
    feature_tracker.img1 = cv2_img
    if feature_tracker.img2 is not None:
        # Detect keypoints and descriptors. Update feature_tracker class with keypoints and descriptors
        matches, good_matches = detect_and_match_features()
        if len(good_matches) > 10:
            src_pts = np.float32([ feature_tracker.kp1[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
            dst_pts = np.float32([ feature_tracker.kp2[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)
            matched_kp_1_kp_2 = {'first': src_pts, 'second': dst_pts}
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            # FInd essential matrix
            E, mask = cv2.findEssentialMat(matched_kp_1_kp_2['first'], matched_kp_1_kp_2['second'], camera_params_.K, cv2.RANSAC, 0.999, 1.0, None)
            # Recover pose
            _, R, t, mask = cv2.recoverPose(E, matched_kp_1_kp_2['first'], matched_kp_1_kp_2['second'], camera_params_.K)
            # Create pose matrix
            relative_pose_matrix = np.zeros((4,4))
            relative_pose_matrix[:3, :3] = R
            relative_pose_matrix[:3, 3] = t.T
            relative_pose_matrix[3, 3] = 1
            # Update relative pose tracker
            pose_tracker.relative_pose = relative_pose_matrix
            # Update pose tracker
            pose_tracker.update_pose()
            pose_tracker.prev_pose = pose_tracker.pose_estimate
            # Publish pose estimation
            pose = Pose()
            pose.position.x = pose_tracker.pose_estimate[0, 3]
            pose.position.y = pose_tracker.pose_estimate[1, 3]
            pose.position.z = pose_tracker.pose_estimate[2, 3]
            # Convert Rotation marix to quaternion
            q = tf.transformations.quaternion_from_matrix(pose_tracker.pose_estimate)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            pub_pose_estimation.publish(pose)
        else: 
            print("Not enough matches are found - %d/%d" % (len(good_matches),10))
            matchesMask = None
            # good_matches = []
        # Draw inliers
        draw_inliers(feature_tracker.img1, feature_tracker.img2, feature_tracker.kp1, feature_tracker.kp2, good_matches, M, matchesMask)
    feature_tracker.img2 = cv2_img


def listener():
    pub = rospy.Publisher('camera_pose', String, queue_size=10)
    rospy.init_node('pose_estimation_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Init camera parameters
    camera_params_.K = np.array([[415.69219381653056, 0.0, 360.0], [0.0, 415.69219381653056, 240.0], [0.0, 0.0, 1.0]])
    camera_params_.D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    cb_in = lambda data: callback(data, feature_tracker)
    rospy.Subscriber("/cam1/image_raw", Image, cb_in)
    
    rospy.spin()

if __name__ == '__main__':
    listener()