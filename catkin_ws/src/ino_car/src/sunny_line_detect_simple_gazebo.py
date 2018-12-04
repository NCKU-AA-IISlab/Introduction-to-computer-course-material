#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_utils.jpg import image_cv_from_jpg  #location:f23-LED/led_detection/include
import threading
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, Image
from ino_car.msg import LaneLine, LaneLines

class LineDetectorNode(object):
    def __init__(self):
        self.node_name = "LineDetectorNode"
        self.verbose = None
        # Thread lock 
        self.thread_lock = threading.Lock()
       
        # Constructor of line detector 
        self.bridge = CvBridge()

        # Publishers
        self.pub_image = rospy.Publisher("~image_with_lines", Image, queue_size=1)
        self.pub_lines = rospy.Publisher("~segment_list",  LaneLines, queue_size=1)       
        # Subscribers
        self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
        #------------------------------------------
        self.bottom_width = 0.85  # width of bottom edge of trapezoid, expressed as percentage of image width
        self.top_width = 0.3  # ditto for top edge of trapezoid
        self.height = 0.4  # height of the trapezoid expressed as percentage of image height
        self.height_from_bottom = 0.05 # height from bottom as percentage of image height
        self.x_translation = -0.02  # Can be +ve or -ve. Translation of midpoint of region of interest along x axis

        self.center =[0,0]
        self.hasleft= False
        self.hasright = False
        self.lanewidth =400
        # -----------------------------------------
        #color
        self.hsv_white1=  np.array([0,0,150])
        self.hsv_white2= np.array([180,50,255])
        self.hsv_yellow1=  np.array([25,120,90])
        self.hsv_yellow2= np.array([45,255,255])
        self.hsv_red1=    np.array([0,140,100])
        self.hsv_red2=    np.array([15,255,255])
        self.hsv_red3=    np.array([165,140,100])
        self.hsv_red4=    np.array([180,255,255])
        self.dilation_kernel_size = 3
        #----------------------------------------
    def _colorFilter(self,hsv):
        # threshold colors in HSV space
        bw_white = cv2.inRange(hsv, self.hsv_white1, self.hsv_white2)
        bw_yellow = cv2.inRange(hsv, self.hsv_yellow1, self.hsv_yellow2)
        bw = cv2.bitwise_or(bw_white, bw_yellow)
        # binary dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.dilation_kernel_size, self.dilation_kernel_size))
        bw = cv2.dilate(bw, kernel)
            
        return bw

    def cbImage(self, image_msg):


        # Start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()  #start execution
        # Returns rightaway
    
    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

    def canny_edge_median(self,img):
        """canny_edge_median takes an image and does auto-thresholding
        using median to compute the edges using canny edge technique
        """
        median = np.median(img)
        low_threshold = median * 0.66
        upper_threshold = median * 1.33
        return cv2.Canny(img, low_threshold, upper_threshold)


    def region_of_interest(self,img, vertices):
        """
        Only keeps the part of the image enclosed in the polygon and
        sets rest of the image to black
        """
        mask = np.zeros_like(img)
        
        mask_color = 255

        # filling pixels inside the polygon defined by "vertices" with the fill color
        cv2.fillPoly(mask, vertices, mask_color)

        # returning the image only where mask pixels are nonzero
        masked_image = cv2.bitwise_and(img, mask)

        return masked_image, mask

    def highway_lane_lines(self,img,img_shape ):
        """
        Computes hough transform, separates lines on left and right side of the highway lane computed
        by hough transform, then forms a single line on the right side and left side
        """

        # Computing lines with hough transform
        lines = self.hough_transform(img)
        if lines is None:
            return None,None,None
        # Removing horizontal lines detected from hough transform
        lane_lines = self.filter_horizontal_lines(lines)
        # Separating lines on left and right side of the highway lane
        left_lines, right_lines = self.separate_lines(lane_lines)

        # Filtering lines i.e. removing left lines that are closer to right side and vice versa
        left_lines, right_lines = self.filter_lane_lines(left_lines, right_lines,)

        # Computing one single line for left and right side
        left_side_line = self.draw_single_line(left_lines,img_shape )
        right_side_line = self.draw_single_line(right_lines,img_shape )
        
        return left_side_line, right_side_line,lines
        #return left_lines, right_lines,lane_lines
    def hough_transform(self,img):
        """
            Computes lines using the probabilistic hough transform provided by OpenCV
            Thus it computes lines of finite size and returns them in form of an array

        :param img: masked edge detected image with only region of interest
        :return:
        """
        # Parameters
        rho = 2  # distance resolution in pixels of the Hough grid
        theta = 1 * np.pi / 18  # angular resolution in radians of the Hough grid
        threshold = 10  # minimum number of votes (intersections in Hough grid cell)
        min_line_length = 10  # m0inimum number of pixels making up a line
        max_line_gap = 15  # maximu gap in pixels between connectable line segments


        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_length,
                                maxLineGap=max_line_gap)
        
        
        return lines


    def filter_horizontal_lines(self,lines):
        """
        Removes all lines with slope between -10 and +10 degrees
        This is done because for highway lane lines the lines will be closer to being
        vertical from the view of the front mounted camera
        """
        if lines is None:
            return
        non_horizontal_lines = [l for l in lines if
                                not -10 <= np.rad2deg(np.arctan(float((l[0][3] - l[0][1])) / 0.0001*float((l[0][2] - l[0][0])))) <= 10]
        non_horizontal_lines = np.array(non_horizontal_lines)
        return non_horizontal_lines




    def separate_lines(self,lines):
        """
        Separates the left and right lines of the highway lane
        :param lines: an array containing the lines which make left and right side of highway lane
        """
        left_lines = []
        right_lines = []
        # Here we separate coordinates of left and right-side lines of the highway lane
        # Since the y-axis is positive in downwards direction and x-axis is positive in right hand direction
        # With origin at the top left corner of the image
        # A negative slope will mean that the line is on the left ( in normal coordinate system it
        # will mean on the right side)
        # A positive slope will mean that the line is on the right ( in normal coordinate system it
        # will mean on the left side)

        for l in lines:
            slope = float((l[0][3] - l[0][1])) / 0.0001+float((l[0][2] - l[0][0]))
            if slope < 0:
                # Slope is negative hence line is on the left side
                left_lines += [(l[0][0], l[0][1], l[0][2], l[0][3])]
            elif slope > 0:
                # Slope is positive hence line is on the right side
                right_lines += [(l[0][0], l[0][1], l[0][2], l[0][3])]
            else:
                print("Something looks fishy here")

        return left_lines, right_lines


    def filter_lane_lines(self,left_lines, right_lines):
        """
        This function removes lines from left_lines that are closer to the right-side of the highway lane
        and from right_lines removes lines that are closer to left-side of highway lane. It also removes
        the lines which are more or less than 10 degrees from the median slope of each side.
        """
        if len(left_lines) == 0 or len(right_lines) == 0:
            return left_lines, right_lines

        # Filtering lines that lie close to the other side, for instance
        # lines in left_lines array that are closer to the right lane line
        x_top_left = []
        for x1, y1, x2, y2 in left_lines:
            x_top_left += [x2]
        x_top_left_median = np.median(x_top_left)
        left_lines_final = [l for l in left_lines if l[2] <= x_top_left_median]

        slope_left_lines = []
        for x1, y1, x2, y2 in left_lines_final:
            slope_left_lines += [np.rad2deg(np.arctan((y2 - y1) / (x2 - x1)))]

        x_top_right = []
        for x1, y1, x2, y2 in right_lines:
            x_top_right += [x1]
        x_top_right_median = np.median(x_top_right)
        right_lines_final = [l for l in right_lines if l[0] >= x_top_right_median]

        slope_right_lines = []
        for x1, y1, x2, y2 in right_lines_final:
            slope_right_lines += [np.rad2deg(np.arctan((y2 - y1)/(x2 - x1)))]

        # Filtering based on slope
        median_left_lines_slope = np.median(slope_left_lines)
        left_lines_final_filtered = []
        for i in range(len(left_lines_final)):
            if (-1 + median_left_lines_slope) <= slope_left_lines[i] <= (10 + median_left_lines_slope):
                left_lines_final_filtered += [left_lines_final[i]]

        median_right_lines_slope = np.median(slope_right_lines)
        right_lines_final_filtered = []
        for i in range(len(right_lines_final)):
            if (-5 + median_right_lines_slope) <= slope_right_lines[i] <= (5 + median_right_lines_slope):
                right_lines_final_filtered += [right_lines_final[i]]

        return left_lines_final_filtered, right_lines_final_filtered


    def draw_single_line(self,lines,img_shape):
        """
        Takes in an array of lines and combines them into a single line
        """
        if len(lines) == 0:
            return None

        # Maximum and minimum y-coordinate for the sigle line on left and right side
        y_max = int(img_shape[0] - img_shape[0] * self.height_from_bottom)
        y_min = int(img_shape[0] - img_shape[0] * self.height_from_bottom) - int(img_shape[0] * self.height)

        # Computing the top and bottom x co-ordinate obtained by extrapolating
        # the limited length lines.
        x_top = []
        x_bottom = []
        for x1, y1, x2, y2 in lines:
            z = np.polyfit([x1, x2], [y1, y2], 1)
            m, c = z
            x_top.append(int((y_min - c) / m))
            x_bottom.append(int((y_max - c) / m))


        x_avg_top = np.int(np.median(x_top))
        x_avg_bottom = np.int(np.median(x_bottom))

        return [x_avg_bottom, y_max, x_avg_top, y_min]

    def compute_mask_vertices(self,img_shape):
        """
        This function takes an image as input, requires the parameters to be set manually
        and generates the coordinates for the mask vertices.
        """

        
        vertices = np.array(
            [[[(img_shape[1] * (1 - self.bottom_width)) // 2, int(img_shape[0] - img_shape[0] * self.height_from_bottom)],
            [int(img_shape[1] *self.bottom_width) + (img_shape[1] * (1 - self.bottom_width)) // 2,
            int(img_shape[0] - img_shape[0] * self.height_from_bottom)],
            [int(img_shape[1] * self.top_width) + (img_shape[1] * (1 - self.top_width)) // 2,
            int(img_shape[0] - img_shape[0] * self.height_from_bottom) - int(img_shape[0] * self.height)],
            [(img_shape[1] * (1 - self.top_width)) // 2,
            int(img_shape[0] - img_shape[0] * self.height_from_bottom) - int(img_shape[0] * self.height)]]],
            dtype=np.int32)

        vertices = np.array(vertices[:] - [self.x_translation * img_shape[1], 0], dtype='int')
        return vertices

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):

            # Return immediately if the thread is locked
            return

        try:
            self.processImage_(image_msg)
        finally:
            # Release the thread lock
            self.thread_lock.release()
    
    def processImage_(self, image_msg):
        # Decode from compressed image with OpenCV
        try:
            image_cv = image_cv_from_jpg(image_msg.data)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return

        # Resize and crop image
        hei_original, wid_original = image_cv.shape[0:2]
        
        gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)

        # Applying gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # color
        hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)
        hsv =self. _colorFilter(hsv)
        
        # Computing edges
        img_edges = self.canny_edge_median(blur)
        
        # Computing region of interest
        img_shape = gray.shape
        my_vertices = self.compute_mask_vertices(img_shape)
        masked_image, mask = self.region_of_interest(img_edges, my_vertices)
        edge_color = cv2.bitwise_and(hsv, masked_image)
        # Computing lane lines
        final_left_line, final_right_line, lane_lines = self.highway_lane_lines(edge_color,img_shape )
        # SegmentList constructor
        segmentList = LaneLines()
        segmentList.header.stamp = image_msg.header.stamp

        image_with_lines = np.copy(image_cv)
        if final_left_line is not None:
            cv2.line(image_with_lines, (final_left_line[0], final_left_line[1]), (final_left_line[2], final_left_line[3]), (0,255, 0), 5)
            segmentList.lanelines.extend(self.toSegmentMsg(final_left_line,LaneLine.LEFT))
            self.hasleft = True

        if final_right_line is not None:
            cv2.line(image_with_lines, (final_right_line[0], final_right_line[1]), (final_right_line[2], final_right_line[3]), (0, 0, 255), 5)
            self.hasright = True
            segmentList.lanelines.extend(self.toSegmentMsg(final_right_line,LaneLine.RIGHT))

        # Publish segmentList
        self.pub_lines.publish(segmentList)
        if lane_lines is not None:
            for i,pl in enumerate(lane_lines):
                cv2.line(image_with_lines, (pl[0][0], pl[0][1]), (pl[0][2], pl[0][3]), (255, 0, 0),2)
        
        if self.hasleft and self.hasright:
            self.center[0] = (final_left_line[0]+final_right_line[0]+final_left_line[2] +final_right_line[2])/4
            self.center[1] = (final_left_line[1]+final_right_line[1]+final_left_line[3] +final_right_line[3])/4
            cv2.circle(image_with_lines, (self.center[0] ,self.center[1]), 3, (0,255,255), thickness=3, lineType=8, shift=0) 
            self.hasleft = False
            self.hasright = False
        if self.hasleft and not self.hasright:
            self.center[0] = (final_left_line[0]+final_left_line[2] )/2 + self.lanewidth/2
            self.center[1] = (final_left_line[1]+final_left_line[3] )/2
            cv2.circle(image_with_lines, (self.center[0] ,self.center[1]), 3, (0,255,255), thickness=3, lineType=8, shift=0) 
            self.hasleft = False
            self.hasright = False
        if not self.hasleft and  self.hasright:
            self.center[0] = (final_right_line[0]+final_right_line[2] )/2 - self.lanewidth/2
            self.center[1] = (final_right_line[1]+final_right_line[3] )/2
            cv2.circle(image_with_lines, (self.center[0] ,self.center[1]), 3, (0,255,255), thickness=3, lineType=8, shift=0) 
            self.hasleft = False
            self.hasright = False
        
        cv2.polylines(image_with_lines,my_vertices,True,(0,255,255))
        # Publish the frame with lines
        image_msg_out = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")
        image_msg_out.header.stamp = image_msg.header.stamp
        self.pub_image.publish(image_msg_out)
    
    def toSegmentMsg(self,  line,  side):
        
        segmentMsgList = []
        
        segment = LaneLine()
        segment.side = side
        segment.pixels_line[0].x = line[0]
        segment.pixels_line[0].y = line[1]
        segment.pixels_line[1].x = line[2]
        segment.pixels_line[1].y = line[3]

        segmentMsgList.append(segment)
        return segmentMsgList

    def onShutdown(self):
        self.loginfo("Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('line_detector',anonymous=False)
    line_detector_node = LineDetectorNode()
    rospy.on_shutdown(line_detector_node.onShutdown)
    rospy.spin()
