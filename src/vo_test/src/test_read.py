import rosbag
import cv2
import rospy
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
from std_msgs.msg import String

# ros_bag_dir = '/home/aaron/datasets/vnav/'
# bridge = CvBridge()

# bag = rosbag.Bag(ros_bag_dir + 'MH_01_easy.bag')


# Published topics:
#  * /rosout_agg [rosgraph_msgs/Log] 1 publisher
#  * /rosout [rosgraph_msgs/Log] 1 publisher
#  * /clock [rosgraph_msgs/Clock] 1 publisher
#  * /imu0 [sensor_msgs/Imu] 1 publisher
#  * /cam1/image_raw [sensor_msgs/Image] 1 publisher
#  * /cam0/image_raw [sensor_msgs/Image] 1 publisher
#  * /leica/position [geometry_msgs/PointStamped] 1 publisher

# def test():
#     for topic, img, t in bag.read_messages(topics=['/cam1/image_raw']):
#         cv2_img = bridge.imgmsg_to_cv2(img, "bgr8")
#         cv2.imshow('image', cv2_img)
#         # Display image without waiting for a key press
#         cv2.waitKey(1)
#     bag.close()

def main():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        rate.sleep()

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass