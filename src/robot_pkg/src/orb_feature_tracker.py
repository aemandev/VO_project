import cv2
class OrbFeatureTracker:
    def __init__(self):
        self.img1 = None
        self.img2 = None
        self.kp1 = None
        self.kp2 = None
        self.descriptors1 = None
        self.descriptors2 = None
        self.detector = cv2.ORB_create()

    def detect_keypoints(self, img) -> list:
        kp_out = self.detector.detect(img)
        return kp_out
    def describe_keypoints(self, img, kp) -> tuple:
        kp, des = self.detector.compute(img, kp)
        return kp, des
    def match_dscriptors(self, des1, des2):
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        matcher = cv2.FlannBasedMatcher(index_params, search_params)
        matches = matcher.knnMatch(des1, des2, k=2)
        good_match = []
        for match in matches:
            if len(match) < 2:
                continue
            if match[0].distance < 0.8 * match[1].distance:
                good_match.append(match[0])
        return matches, good_match