
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import ColorRGBA

from utils.calc_quaternion_from_euler import calc_quaternion_from_euler

from geometry_msgs.msg import Point, TransformStamped
import numpy as np
class VisualizerROS:
    def __init__(self):
        ## Map Elements
        self.elem_pub = rospy.Publisher('/map_elements', MarkerArray, queue_size=1)
        self.elem_msg = MarkerArray()
        self.elem_gen = self.MarkerGenerator()

        ## Pose
        self.pose_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
        self.pose_msg = PoseStamped()

        ## Pose Marker
        self.pose_marker_pub = rospy.Publisher('/pose_marker', TransformStamped, queue_size=10)
        self.pose_marker_msg = TransformStamped()

    def publish(self):
        ## Publish ROS message
        self.elem_pub.publish(self.elem_msg)
        self.pose_pub.publish(self.pose_msg)
        self.pose_marker_pub.publish(self.pose_marker_msg)

        ## Reset ROS message
        self.elem_msg = MarkerArray()
        self.pose_msg = PoseStamped()
        self.pose_marker_msg = TransformStamped()
        print('-------pub!~!!')

    def add_elements(self, elems_WC, element_name):
        if element_name == "LINEMARK":


            id_ = '0'
            n = 1
            color = (1.0, 1.0, 1.0, 0.5)
            marker = self.elem_gen.Points(element_name, 0, 1, color)

            for i in range(elems_WC.shape[1]): # 414
                marker.points.append(Point(x=elems_WC[0, i],
                                            y=elems_WC[1, i],
                                            z=elems_WC[2, i],                                          
                                           ))

            self.elem_msg.markers.append(marker)

    def add_pose(self, pose_arr):
        # Set the position
        stamp = rospy.Time.now()

        pose = Pose()
        pose.position.x = pose_arr[0]
        pose.position.y = pose_arr[1]
        pose.position.z = pose_arr[2]

        # Set the orientation
        quaternion = calc_quaternion_from_euler(pose_arr[3:6])

        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]


        self.pose_msg.header.stamp = stamp
        self.pose_msg.header.frame_id = 'map'
        self.pose_msg.pose = pose
        

        ## Pose Marker

    # Set the translation and rotation values in the transform message
        self.pose_marker_msg.header.frame_id = 'body'
        self.pose_marker_msg.header.stamp = stamp
        # self.pose_marker_msg.child_frame_id = 'base_link'
        self.pose_marker_msg.transform.translation.x = pose_arr[0]
        self.pose_marker_msg.transform.translation.y = pose_arr[1]
        self.pose_marker_msg.transform.translation.z = pose_arr[2]

        self.pose_marker_msg.transform.rotation.w = quaternion[0]
        self.pose_marker_msg.transform.rotation.x = quaternion[1]
        self.pose_marker_msg.transform.rotation.y = quaternion[2]
        self.pose_marker_msg.transform.rotation.z = quaternion[3]


        # self.pose_marker_msg.header.frame_id = "map"
        # self.pose_marker_msg.type = self.pose_marker_msg.LINE_LIST
        # self.pose_marker_msg.action = self.pose_marker_msg.ADD
        # self.pose_marker_msg.scale.x = 0.02
        # self.pose_marker_msg.scale.y = 0.02
        # self.pose_marker_msg.pose = pose

        # r = 1.0
        # g = 1.0
        # b = 1.0
        # a = 1.0

        # # self.pose_marker_msg.color = ColorRGBA(r, g, b, a)

        # x, y, z, rx, ry, rz = pose_arr[:]

        # axis = np.eye(3)
        # for i in range(3):
        #     start = np.array([x, y, z])
        #     end = start + axis[i, :]
        #     self.pose_marker_msg.points.append(Point(*start))
        #     self.pose_marker_msg.points.append(Point(*end))
        #     self.pose_marker_msg.colors.append(ColorRGBA(1,1,1,1))
        #     self.pose_marker_msg.colors.append(ColorRGBA(1,1,1,1))






    class MarkerGenerator:

        def __init__(self):
            pass


        def Bound(self, ns : str, id_ : str, n : int, points : list,  color):

            marker = self.Points(f'{ns}_{id_}', n, 0.1, color)
            for pt in points:
                marker.points.append(Point(x=pt[0], y=pt[1], z=pt[2]))

            return marker

        def Line(self, ns, id_, scale, color):
            marker = Marker()
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.header.frame_id = 'map'
            marker.ns = ns
            marker.id = id_
            marker.lifetime = rospy.Duration(0)
            marker.scale.x = scale
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            return marker

        def Points(self, ns, id_, scale, color):
            marker = Marker()
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.header.frame_id = 'map'
            marker.ns = ns
            marker.id = id_
            marker.lifetime = rospy.Duration(0)
            marker.scale.x = scale
            marker.scale.y = scale
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            return marker