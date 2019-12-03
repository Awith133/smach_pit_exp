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
    marker.type = Marker.ARROW

    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1
    marker.scale.y = 0.01
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

        for i in range(0, num_cameras):
            marker = get_camera_marker(i)
            marker.pose.position.x = 0.511108
            marker.pose.position.y = 0.511498
            marker.pose.position.z = -1.93398
            marker.pose.orientation.x = 0.9498423
            marker.pose.orientation.y = 0.0291373
            marker.pose.orientation.z = -0.2909317
            marker.pose.orientation.w = 0.1109474
            pub.publish(marker)



if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
