import json
import time
import sys
import cv2
import numpy
import math
from enum import Enum
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSource, CvSink, MjpegServer, VideoMode
from ntcore import NetworkTableInstance, EventFlags

# time.sleep(60) #Waits 1 minute for the RoboRio to boot.

ntinst = NetworkTableInstance.getDefault() #Creates a NetworkTableInstance object.

# Creating a Network Tables server. Server Name: "wpilibpi"
#ntinst.startServer()

# Connecting to Network Tables as a client.
ntinst.startClient4("wpilibpi")
ntinst.setServerTeam(8051)
ntinst.startDSClient()

# Publishing a table called "Vision" Network Tables. Values published to "Vision" appear on the dashboard.
table = ntinst.getTable("Vision")
timePub = table.getDoubleTopic("Time").publish() # Creates a topic called "Time" that accepts doubles.
framePub = table.getDoubleTopic("Frame").publish() # Creates a topic called "Frame" that accepts doubles.
xPub = table.getDoubleArrayTopic("x").publish() # Creates a topic called "x" that accepts double arrays.
yPub = table.getDoubleArrayTopic("y").publish() # Creates a topic called "y" that accepts double arrays.
sizePub = table.getDoubleArrayTopic("Size").publish() # Creates a topic called "Size" that accepts double arrays.

# Initializes the camera and begins streaming video to Network Tables.
camera = CameraServer.startAutomaticCapture() # USB Camera object.
camera.setResolution(320,240) #lower resolutions are faster to process and less bandwidth
camera.setFPS(30) #30 is the max
camera.setExposureManual(45) #Range 0-100. Sets how much light makes it to the sensor.
camera.setWhiteBalanceManual(3300) # Sets the warmth of the image. Lower values are colder/bluer and higher values are warmer/oranger
camera.setBrightness(30) #WPILIB bug. Camera will always reset to a brightness of 0. Keep this value at 0 for consistency.

sink = CameraServer.getVideo() # CvSink object. Frames from the camera are sent here to be processed.
source = CameraServer.putVideo("Vision Output", 160, 120) # CvSource object. Output frames from CV are sent here to be uploaded to Network Tables.
img_in = numpy.zeros(shape=(160,120,3),dtype=numpy.uint8) # creates a 3D array size 320 x 240 x 3. Each element is an unsigned 8 bit integer (0-255). Value initialized to 0.
img_out = numpy.copy(img_in) # Creates another array for the output image.

# Initializes frame and start time variables to be published to Network Tables.
frame = 0
start_time = time.time()

# GRIP generated code. Copy/pasted verbatim.
class GripPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """
    
    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__blur_type = BlurType.Gaussian_Blur
        self.__blur_radius = 3

        self.blur_output = None

        self.__hsv_threshold_input = self.blur_output
        self.__hsv_threshold_hue = [60, 100]
        self.__hsv_threshold_saturation = [100, 255]
        self.__hsv_threshold_value = [70, 255]

        self.hsv_threshold_output = None

        self.__find_blobs_input = self.hsv_threshold_output
        self.__find_blobs_min_area = 20.0
        self.__find_blobs_circularity = [0.0, 1.0]
        self.__find_blobs_dark_blobs = False

        self.find_blobs_output = None


    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step Blur0:
        self.__blur_input = source0
        (self.blur_output) = self.__blur(self.__blur_input, self.__blur_type, self.__blur_radius)

        # Step HSV_Threshold0:
        self.__hsv_threshold_input = self.blur_output
        (self.hsv_threshold_output) = self.__hsv_threshold(self.__hsv_threshold_input, self.__hsv_threshold_hue, self.__hsv_threshold_saturation, self.__hsv_threshold_value)

        # Step Find_Blobs0:
        self.__find_blobs_input = self.hsv_threshold_output
        (self.find_blobs_output) = self.__find_blobs(self.__find_blobs_input, self.__find_blobs_min_area, self.__find_blobs_circularity, self.__find_blobs_dark_blobs)


    @staticmethod
    def __blur(src, type, radius):
        """Softens an image using one of several filters.
        Args:
            src: The source mat (numpy.ndarray).
            type: The blurType to perform represented as an int.
            radius: The radius for the blur as a float.
        Returns:
            A numpy.ndarray that has been blurred.
        """
        if(type is BlurType.Box_Blur):
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif(type is BlurType.Gaussian_Blur):
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif(type is BlurType.Median_Filter):
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))

    @staticmethod
    def __hsv_threshold(input, hue, sat, val):
        """Segment an image based on hue, saturation, and value ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max value.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
        return cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))

    @staticmethod
    def __find_blobs(input, min_area, circularity, dark_blobs):
        """Detects groups of pixels in an image.
        Args:
            input: A numpy.ndarray.
            min_area: The minimum blob size to be found.
            circularity: The min and max circularity as a list of two numbers.
            dark_blobs: A boolean. If true looks for black. Otherwise it looks for white.
        Returns:
            A list of KeyPoint.
        """
        params = cv2.SimpleBlobDetector_Params()
        params.filterByColor = 1
        params.blobColor = (0 if dark_blobs else 255)
        params.minThreshold = 10
        params.maxThreshold = 220
        params.filterByArea = True
        params.minArea = min_area
        params.filterByCircularity = True
        params.minCircularity = circularity[0]
        params.maxCircularity = circularity[1]
        params.filterByConvexity = False
        params.filterByInertia = False
        detector = cv2.SimpleBlobDetector_create(params)
        return detector.detect(input)


BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

# Creates a GripPipeline object using the uploaded grip.py code.
pipeline = GripPipeline()

# Loops infintely.
while True:
    frame_time, img_in = sink.grabFrame(img_in) # Grabs the current frame. The frame is stored in the "img_in" array.

    # "img_in" is passed to the GRIP pipeline to be processed
    pipeline.process(img_in)

    # Updates the frame and time variables, then publishes their new values to Network Tables.
    timePub.set(time.time() - start_time)
    frame = frame + 1
    framePub.set(frame)
    
    keypoints = pipeline.find_blobs_output # Grabs the keypoints from the blob detection algorithm and stores them in "keypoints" Variable type: cv2.keypoints
    img_out = cv2.drawKeypoints(pipeline.hsv_threshold_output, keypoints, img_out, (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) #Draws circles corresponding to blobs on the output image.

    # Initializes the arrays to store blob posiiton and size information
    xs = []
    ys = []
    sizes = []
    
    # Extracts the blob position and time information from the keypoints object
    for point in keypoints:
        x = point.pt[0]
        y = point.pt[1]
        size = point.size

        xs.append(x)
        ys.append(y)
        sizes.append(size)

    # Publishes the blob position and time information to Network Tables.
    if len(xs) > 0:
        xPub.set(xs)
        yPub.set(ys)
        sizePub.set(sizes)

    # the output of the blur operation is taken from the GRIP pipeline and output to Network Tables.
    source.putFrame(img_out) 