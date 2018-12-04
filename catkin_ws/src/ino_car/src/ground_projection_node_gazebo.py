#!/usr/bin/env python

import rospy
import cv2
from ground_projection.srv import EstimateHomography, EstimateHomographyResponse, GetGroundCoord, GetGroundCoordResponse, GetImageCoord, GetImageCoordResponse
from duckietown_msgs.msg import (Pixel, Vector2D, Segment, SegmentList)
from sensor_msgs.msg import (Image, CameraInfo)
from cv_bridge import CvBridge
import numpy as np
from GroundProjection1 import GroundProjection
from geometry_msgs.msg import Point,Vector3
from ino_car.msg import LaneLine, LaneLines
import rviz_tools_py as rviz_tools
import math
class GroundProjectionNode(object):

    def __init__(self):
        self.node_name="Ground Projection"
        self.active = True
        self.bridge=CvBridge()
        self.hasleft = False
        self.hasright = False
        self.lookahead_dis = 0.3
        self.robot_name = rospy.get_param("~config_file_name","robot_not_specified")
        self.markers = rviz_tools.RvizMarkers(self.robot_name + '/base_footprint', 'visualization_marker')
        self.path =[]
        self.gp = GroundProjection(self.robot_name)
        camera_info_topic = "/"+self.robot_name+"/camera_node/camera_info"
        rospy.loginfo("camera info topic is " + camera_info_topic)
        rospy.loginfo("waiting for camera info")
        camera_info = rospy.wait_for_message(camera_info_topic,CameraInfo)
        rospy.loginfo("camera info received")

        self.gp.initialize_pinhole_camera_model(camera_info)
        # Params
        self.hasleft = False
        self.hasright = False
        self.goalpoint = [0,0]
        self.lane_width = 0.1
        self.gp.robot_name = self.robot_name
        self.gp.rectified_input_ = rospy.get_param("rectified_input", False)
        self.image_channel_name = "image_raw"

        # Subs and Pubs
        self.pub_lineseglist_ = rospy.Publisher("~goal_point",Point, queue_size=1)
        self.sub_lineseglist_ = rospy.Subscriber("~lineseglist_in",LaneLines, self.lineseglist_cb)

        # TODO prepare services
        self.service_homog_ = rospy.Service("~estimate_homography", EstimateHomography, self.estimate_homography_cb)
        self.service_gnd_coord_ = rospy.Service("~get_ground_coordinate", GetGroundCoord, self.get_ground_coordinate_cb)
        self.service_img_coord_ = rospy.Service("~get_image_coordinate", GetImageCoord, self.get_image_coordinate_cb)


    def rectifyImage(self,img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="mono8")
        except CvBridgeError as e:
            logger.error(e)
        return gp.rectify(cv_image)

    def lineseglist_cb(self,seglist_msg):
         
        goal_out = Point()

        new_seglist = LaneLines()
        for received_segment in seglist_msg.lanelines:

            if received_segment.side == 0: #LEFT
                self.hasleft = True
            elif received_segment.side == 1: #RIGHT
                self.hasright = True
            new_segment = LaneLine()
            new_segment.points[0] = self.gp.vector2ground(received_segment.pixels_line[0])
            new_segment.points[1] = self.gp.vector2ground(received_segment.pixels_line[1])
            new_segment.normal_vec = self.normal_vector(new_segment.points[0],new_segment.points[1])
            new_segment.points[0].x 
            new_segment.points[1].x   # add the distance between chasis and camera in x-dir
 
            new_seglist.lanelines.append(new_segment)
            self.markers.publishLine(new_segment.points[1], new_segment.points[0], 'purple', 0.01, 0.1) # point1, point2, color, width, lifetime
            self.markers.publishSphere(new_segment.points[1],  'orange', Vector3(0.01,0.01,0.01), 1.0)
        #calculate goal point
        if self.hasleft and self.hasright: 
            goal_out.x = (new_seglist.lanelines[1].points[0].x + new_seglist.lanelines[1].points[1].x+new_seglist.lanelines[0].points[0].x + new_seglist.lanelines[0].points[1].x)/4
            goal_out.y = (new_seglist.lanelines[1].points[0].y + new_seglist.lanelines[1].points[1].y+new_seglist.lanelines[0].points[0].y + new_seglist.lanelines[0].points[1].y)/4

        if self.hasleft and not self.hasright: 
            goal_out.x = (new_seglist.lanelines[0].points[0].x + new_seglist.lanelines[0].points[1].x)/2
            goal_out.y = (new_seglist.lanelines[0].points[0].y + new_seglist.lanelines[0].points[1].y)/2

            goal_out.y = goal_out.y + new_seglist.lanelines[0].normal_vec[1]* self.lane_width
            goal_out.x = goal_out.x + new_seglist.lanelines[0].normal_vec[0]* self.lane_width

        if not self.hasleft and  self.hasright: 
            goal_out.x = (new_seglist.lanelines[0].points[0].x + new_seglist.lanelines[0].points[1].x)/2
            goal_out.y = (new_seglist.lanelines[0].points[0].y + new_seglist.lanelines[0].points[1].y)/2
            
            goal_out.y = goal_out.y - new_seglist.lanelines[0].normal_vec[1]* self.lane_width
            goal_out.x = goal_out.x - new_seglist.lanelines[0].normal_vec[0]* self.lane_width

          
        # Publish a sphere using a ROS Point
        point = goal_out
        self.path.append(point)
        self.markers.publishSphere(point,  'orange', Vector3(0.03,0.03,0.03), 20.0) # pose, color

        

        self.pub_lineseglist_.publish(goal_out)
        self.hasleft = False
        self.hasright = False
    
    def normal_vector(self,p1,p2):
        d = math.sqrt((p1.x-p2.x)**2+(p1.y-p2.y)**2)
        n_v = [-(p1.y-p2.y)/d, (p1.x-p2.x)/d]
        return n_v 


    def get_ground_coordinate_cb(self,req):
        return GetGroundCoordResponse(self.gp.pixel2ground(req.normalized_uv))

    def get_image_coordinate_cb(self,req):
        return GetImageCoordResponse(self.gp.ground2pixel(req.gp))

    def estimate_homography_cb(self,req):
        rospy.loginfo("Estimating homography")
        rospy.loginfo("Waiting for raw image")
        img_msg = rospy.wait_for_message("/"+self.robot_name+"/camera_node/image",Image)
        rospy.loginfo("Got raw image")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg,desired_encoding="bgr8")
	    
        except CvBridgeError as e:
            rospy.logerr(e)
        self.gp.estimate_homography(cv_image)
        rospy.loginfo("wrote homography")
        return EstimateHomographyResponse()

    def onShutdown(self):
        rospy.loginfo("[GroundProjectionNode] Shutdown.")

   

if __name__ == '__main__':
    rospy.init_node('ground_projection',anonymous=False)
    ground_projection_node = GroundProjectionNode()
    rospy.on_shutdown(ground_projection_node.onShutdown)
    rospy.spin()
