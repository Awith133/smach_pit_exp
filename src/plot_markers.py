#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import pdb


def get_camera_marker(id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "basic_shapes"
    marker.id = id

    marker.action = Marker.ADD
    marker.type = Marker.CUBE

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.01

    # // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration(0)
    return marker

def publisher():
    pub = rospy.Publisher('camera_poses', Marker, queue_size=10)
    rospy.init_node('Publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    num_cameras = 10
    while not rospy.is_shutdown():

        marker = get_camera_marker(0)
        marker.pose.position.x = -0.6623
        marker.pose.position.y = 1.45197
        marker.pose.position.z = -5.62406
        marker.pose.orientation.x = 0.9498423
        marker.pose.orientation.y = 0.0291373
        marker.pose.orientation.z = -0.2909317
        marker.pose.orientation.w = 0.1109474
        pub.publish(marker)

        marker = get_camera_marker(1)
        marker.pose.position.x = 0.511108
        marker.pose.position.y = 0.511498
        marker.pose.position.z = -1.93398
        marker.pose.orientation.x =  0.99284175
        marker.pose.orientation.y = -0.00221396
        marker.pose.orientation.z = -0.06318781
        marker.pose.orientation.w = 0.10132934
        pub.publish(marker)

        marker = get_camera_marker(2)
        marker.pose.position.x = -0.700784
        marker.pose.position.y = -1.11197
        marker.pose.position.z = 4.21761
        marker.pose.orientation.x = -0.11566806
        marker.pose.orientation.y = -0.11223317
        marker.pose.orientation.z = -0.01673735
        marker.pose.orientation.w = 0.98678493
        pub.publish(marker)

        marker = get_camera_marker(3)
        marker.pose.position.x = 0.477207
        marker.pose.position.y = 0.476803
        marker.pose.position.z = -1.7874
        marker.pose.orientation.x = 0.9498423
        marker.pose.orientation.y = 0.0291373
        marker.pose.orientation.z = -0.2909317
        marker.pose.orientation.w = 0.1109474
        pub.publish(marker)

        marker = get_camera_marker(4)
        marker.pose.position.x = -0.336991
        marker.pose.position.y = -0.7739
        marker.pose.position.z = 2.88306
        marker.pose.orientation.x = 0.9498423
        marker.pose.orientation.y = 0.0291373
        marker.pose.orientation.z = -0.2909317
        marker.pose.orientation.w = 0.1109474
        pub.publish(marker)

        marker = get_camera_marker(5)
        marker.pose.position.x = 0.0829232
        marker.pose.position.y = 1.03344
        marker.pose.position.z = -3.93012
        marker.pose.orientation.x = -0.11566806
        marker.pose.orientation.y = -0.11223317
        marker.pose.orientation.z = -0.01673735
        marker.pose.orientation.w = 0.98678493
        pub.publish(marker)

        marker = get_camera_marker(6)
        marker.pose.position.x = 0.532226
        marker.pose.position.y = -0.515539
        marker.pose.position.z = 1.98572
        marker.pose.orientation.x =  0.99284175
        marker.pose.orientation.y = -0.00221396
        marker.pose.orientation.z = -0.06318781
        marker.pose.orientation.w = 0.10132934
        pub.publish(marker)

        marker = get_camera_marker(7)
        marker.pose.position.x = -0.607216
        marker.pose.position.y = -1.04369
        marker.pose.position.z =  3.93675
        marker.pose.orientation.x = 0.9498423
        marker.pose.orientation.y = 0.0291373
        marker.pose.orientation.z = -0.2909317
        marker.pose.orientation.w = 0.1109474
        pub.publish(marker)

        marker = get_camera_marker(8)
        marker.pose.position.x = 0.347533
        marker.pose.position.y = 0.179788
        marker.pose.position.z = -0.648746
        marker.pose.orientation.x = -0.11566806
        marker.pose.orientation.y = -0.11223317
        marker.pose.orientation.z = -0.01673735
        marker.pose.orientation.w = 0.98678493
        pub.publish(marker)

        marker = get_camera_marker(9)
        marker.pose.position.x = 0.132866
        marker.pose.position.y = 1.53116
        marker.pose.position.z = -5.91487
        marker.pose.orientation.x =  0.99284175
        marker.pose.orientation.y = -0.00221396
        marker.pose.orientation.z = -0.06318781
        marker.pose.orientation.w = 0.10132934
        pub.publish(marker)

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
