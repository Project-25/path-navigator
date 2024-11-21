#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ CL_1336 ]
# Author List:		[ Ashutosh Singh , Danish , Narendra Prajapati ]
# Filename:		    task1a.py
# Functions:
#			        [ detect_aruco() ,calculate_rectangle_area() ,__init__() ,depthimagecb() ,colorimagecb() , process_image()]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /camera/color/image_raw ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
from cv2 import aruco
import math
import tf2_ros
from tf2_ros import TransformException
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import scipy.spatial.transform as transform
from sensor_msgs.msg import CompressedImage, Image
from tf_transformations import euler_from_quaternion,quaternion_from_euler



##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    
    area = None
    width = None

    ############ ADD YOUR CODE HERE ############
    # print(coordinates[0])
    # rect= cv2.minAreaRect(coordinates)     # minimum area of rectangle  it return (center(x,y),(width,height),angle of rotation)
    # box = cv2.boxPoints(rect)               # to make the 4 corners using  rect data and it is same as corners

    area = cv2.contourArea(coordinates)             # it is use to find the area using the four corners
    width = np.sqrt((coordinates[0][3][0] - coordinates[0][0][0])**2 + (coordinates[0][3][1] - coordinates[0][0][1])**2)
    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################


    return area, width


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############


    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    ############ ADD YOUR CODE HERE ############
    
    gray_image=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_parameter=cv2.aruco.DetectorParameters()

    marker_corners, marker_IDs , reject = aruco.detectMarkers(gray_image,aruco_dict,parameters=aruco_parameter)

    if marker_corners:
        rvec, tvec, _ =cv2.aruco.estimatePoseSingleMarkers(marker_corners,size_of_aruco_m,cam_mat,dist_mat)
        total_aruco_markers=range(0,marker_IDs.size)
        for id,corners,itrate in zip(marker_IDs,marker_corners,total_aruco_markers):
            
            
            area,width=calculate_rectangle_area(corners)
            if area>=aruco_area_threshold :
                cv2.polylines(image,[corners.astype(np.int32)],True,(0,255,0),2,cv2.LINE_AA)

                corners=corners.reshape(4,2)
                corners=corners.astype(int)
                

                x_sum = corners[0][0]+ corners[1][0]+ corners[2][0]+ corners[3][0]
                y_sum = corners[0][1]+ corners[1][1]+ corners[2][1]+ corners[3][1]
                    
                x_centerPixel = int(x_sum*.25)
                y_centerPixel = int(y_sum*.25)
                centre_pt=[x_centerPixel,y_centerPixel]
                # x_centerPixel = int(center[0])
                # y_centerPixel = int(center[1])
                # centre_pt=[x_centerPixel,y_centerPixel]

                center,size,angle=cv2.minAreaRect(corners)
                # center_aruco_list[itrate]=center
                # angle_aruco_list[itrate]=angle
                # width_aruco_list[itrate]=size
                ids.append(id)
                center_aruco_list.append(centre_pt)
                angle_aruco_list.append(rvec[itrate][0][2])
                distance_from_rgb_list.append(tvec[itrate][0][2])
                width_aruco_list.append(width)
                cv2.drawFrameAxes(image,cam_mat,dist_mat,rvec[itrate],tvec[itrate],0.2)
                cv2.putText(image,"center",(x_centerPixel-20,y_centerPixel-20),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)
                cv2.putText(image,f"ids:{id}",centre_pt,cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0),1,cv2.LINE_AA) 
                cv2.circle(image,centre_pt,4,(203, 192, 255),-1)        #to draw a point on the centre of each detected aruco marker
    cv2.imshow("camera",image)
        # cv2.waitKey(0)
        
    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection

    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 

    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################
    # print(angle_aruco_list)
    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        self.aruco_data=None
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        try:
            self.depth_image =self.bridge.imgmsg_to_cv2(data,desired_encoding="16UC1")
            # cv2.imshow("depth_image",cv_depth)
            # cv2.waitKey(1)
            # print(cv_depth.shape)

        except CvBridgeError as e:
            print(f"CvBridgeError:{e}")
            # rclpy.logerr(e)

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        try:
            cv_image =self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            # cv2.imshow("image",cv_image)      #to check if the image is converted or  not
            cv2.waitKey(1)                      
            self.aruco_data=detect_aruco(cv_image)

        except CvBridgeError as e:
            print(f"CvBridgeError:{e}")
            # rclpy.logerr(e)
            

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        cX=0
        cY=0
        

        ############ ADD YOUR CODE HERE ############
        try:
            if self.aruco_data==None:
                return
            center_aruco_list=self.aruco_data[0]
            distance_from_rgb_list=self.aruco_data[1]
            angle_aruco_list=self.aruco_data[2]
            width_aruco_list=self.aruco_data[3]
            ids=self.aruco_data[4]
            
            for iterate in range(0,len(ids)):
                aruco_center=center_aruco_list[iterate]
                # distance_from_rgb=distance_from_rgb_list[iterate]
                angle_aruco=angle_aruco_list[iterate]
                
                width_aruco=width_aruco_list[iterate]
                id=ids[iterate]
                angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)
                
                #quaternions calculation

                roll_radian=0.0             #angle of roll in degree
                pitch_radian=0.0            #angle of pitch in degree
                yaw_radian=angle_aruco      #angle of yaw in degree

                

                quaternion = quaternion_from_euler(roll_radian,pitch_radian,yaw_radian)     # the quaternion var will give a list of [x ,y ,z ,w]

                cX=aruco_center[0]
                cY=aruco_center[1]
                if cX==None:
                    continue
                if cY==None:
                    continue
                
                distance_from_rgb=self.depth_image[cY,cX] /1000
                if distance_from_rgb==None:
                    continue
                x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
                y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
                z = distance_from_rgb
                # print(x,y,z)
                tf_camera_to_aruco = TransformStamped()
                tf_camera_to_aruco.header.stamp = self.get_clock().now().to_msg()
                tf_camera_to_aruco.header.frame_id = 'camera_link'
                tf_camera_to_aruco.child_frame_id = 'cam_'+ str(id[0])

                tf_camera_to_aruco.transform.translation.x = z
                tf_camera_to_aruco.transform.translation.y = x
                tf_camera_to_aruco.transform.translation.z = y
                
                # tf_camera_to_aruco.transform.rotation.x = quaternion[0]
                # tf_camera_to_aruco.transform.rotation.y = quaternion[1]
                # tf_camera_to_aruco.transform.rotation.z = quaternion[2]
                # tf_camera_to_aruco.transform.rotation.w = quaternion[3]
                tf_camera_to_aruco.transform.rotation.x = 0.0
                tf_camera_to_aruco.transform.rotation.y = 0.0
                tf_camera_to_aruco.transform.rotation.z = 0.0
                tf_camera_to_aruco.transform.rotation.w = 1.0
                
                self.br.sendTransform(tf_camera_to_aruco)
                
                try:
                    transform=None
                    transform=self.listener.buffer.lookup_transform('base_link',tf_camera_to_aruco.child_frame_id,rclpy.time.Time())
                    # print(transform)
                    tf_base_to_aruco = TransformStamped()
                    tf_base_to_aruco.header.stamp = self.get_clock().now().to_msg()
                    tf_base_to_aruco.header.frame_id = 'base_link'
                    tf_base_to_aruco.child_frame_id = 'obj_'+ str(id[0])

                    # tf_base_to_aruco.transform.translation.x = -transform.transform.translation.x
                    # tf_base_to_aruco.transform.translation.y = -transform.transform.translation.y
                    # tf_base_to_aruco.transform.translation.z = -transform.transform.translation.z
                    tf_base_to_aruco.transform.translation.x = transform.transform.translation.x
                    tf_base_to_aruco.transform.translation.y = transform.transform.translation.y
                    tf_base_to_aruco.transform.translation.z = transform.transform.translation.z
                    # print(transform.transform.translation.z)
                    # tf_base_to_aruco.transform.rotation.x = transform.transform.rotation.x
                    # tf_base_to_aruco.transform.rotation.y = transform.transform.rotation.y
                    # tf_base_to_aruco.transform.rotation.z = transform.transform.rotation.z
                    # tf_base_to_aruco.transform.rotation.w = transform.transform.rotation.w
                    r,p,y=euler_from_quaternion(quaternion)
                    r=r+1.54
                    p=p+3.14
                    y=y+1.54
                    quaternion=quaternion_from_euler(-r,p,-y)
                    print(id)
                    # print(r,p,y)
                    
                    tf_base_to_aruco.transform.rotation.x = quaternion[0]
                    tf_base_to_aruco.transform.rotation.y = quaternion[1]
                    tf_base_to_aruco.transform.rotation.z = quaternion[2]
                    tf_base_to_aruco.transform.rotation.w = quaternion[3]
                    
                    print(tf_base_to_aruco.transform.translation.x, tf_base_to_aruco.transform.translation.y, tf_base_to_aruco.transform.translation.z)
                    print(tf_base_to_aruco.transform.rotation.x, tf_base_to_aruco.transform.rotation.y, tf_base_to_aruco.transform.rotation.z,tf_base_to_aruco.transform.rotation.w)

                    self.br.sendTransform(tf_base_to_aruco)
                except TransformException as ex:
                    self.get_logger().info(f'Could not transform: {ex}')

        except Exception as e:
            print(e)
            
                
        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1B)
        #               Also, auto eval script will be judging angular difference aswell. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()
