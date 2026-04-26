#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
import math
import rospkg

def publish_city_mesh():
    rospy.init_node('city_rviz_publisher')
    pub = rospy.Publisher('/city_mesh_outline', Marker, queue_size=1, latch=True)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "city_environment"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD

    # Direct local path to the mesh file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('uav_city_navigation') # <--- UPDATE THIS
    marker.mesh_resource = f"file://{pkg_path}/models/city_osm_roundabout/meshes/model.dae"

    # Exact pose matching the Gazebo .world file
    # Match the offset from the .world file
    marker.pose.position.x = -6.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # Convert Yaw (0.083366 rad) to Quaternion
    yaw = 0.083366
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = math.sin(yaw / 2.0)
    marker.pose.orientation.w = math.cos(yaw / 2.0)

    # Apply the 10x Skyscraper scale
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 1.0

    # Set color to light gray and 50% transparent
    marker.color.r = 0.7
    marker.color.g = 0.7
    marker.color.b = 0.7
    marker.color.a = 0.5 

    # Wait for subscribers to connect then publish
    rospy.sleep(1.0)
    pub.publish(marker)
    rospy.loginfo("City outline published to RViz on topic: /city_mesh_outline")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_city_mesh()
    except rospy.ROSInterruptException:
        pass
