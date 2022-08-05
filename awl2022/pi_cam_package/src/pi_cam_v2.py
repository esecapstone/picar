#!/usr/bin/env python3
import time
import picamera
import rospy
import numpy as np
import cv2
from time import sleep
from std_msgs.msg import Float32MultiArray

# fov angle in degrees on x and y axi for pi-cam v2
xFov = 62.2
yFov = 48.8


#if using pi-cam v1, comment out v2 fov settings above,
#then uncomment the v1 settings below

# fov angle in degrees on x and y axi for pi-cam v1

# xFov = 53.50
# yFov = 41.41



# actual resolution of img (x and y) after downscaling this is full resolution (does not crop the image!)
# Note that xRes needs to be multiple of 32 and yRes needs to be multiple of 16 
#Res = 1280
#yRes = 720
xRes = 1600
yRes = 912


# calculate angle value of one pixel
xPixAngle = xFov / xRes
yPixAngle = yFov / yRes

# finding center of image
halfX = (int)(xRes / 2)
halfY = (int)(yRes / 2)
y_top = 0
y_bot = yRes


#Initializing Variables for navigate()
#*** You can simply change the values below (until next set of asterisk marker) to adapt to your needs
num_sample = 64
half_sample = num_sample / 2
sample_dim = 10 # 10x10 pixel areas for sample
sample_threshold = (int)(255 * 0.4 * (sample_dim) ** 2) # at least 40% pixels are white (value 255) to be considered road
intersect_span = 12 #Taking 12 samples on the left and 12 samples on the right for intersection detection
large_lean_left = 28 #if by the 28/64 sample, road is still not spotted, the car is leaning too much left. Turn off intersection detection
large_lean_right = 35 #if by the 35/64 sample, road is still not spotted, the car is leaning too much right. Turn off intersection detection

left_turn_only = True
right_turn_only = False
calibrator_dim = 1*sample_dim

#reading 10x10 pizel square area of img for real-time calibration
calib_row = (int) ((yRes*4)/5 - calibrator_dim / 2)
calib_col = (int) (xRes/2 - calibrator_dim / 2)

edge_threshold = 3 # must have 3 continous road samples to be considered road edge

left_intersect_end = intersect_span #using 1st-12th sample for left intersection
right_intersect_begin = num_sample - intersect_span #using 52nd-64th sample for right intersection

# cropping road
cropped_yRes = 4 * sample_dim
#Intersection Detection uses top 10 pixels, 
yCropTop = int(halfY + (1/4) * yRes) #mask starting at 3/4 of the way down the image
height_btwn_samples = cropped_yRes - 2 * sample_dim
yCropBot = yCropTop + cropped_yRes

#***


left_col = (int) ((1/2 * (1/num_sample) * xRes)-(sample_dim / 2)) #top left corner of the leftmost course correction sample 
left2right_col = (int) ( (1/num_sample)* xRes) #distance between the center of each sample
region_width = (int) (xRes / num_sample)
region_angle = region_width * xPixAngle

# wait for camera to warm up
time.sleep(2)


def calibrate():
    with picamera.PiCamera() as camera:

        camera.resolution = (xRes, yRes)
        camera.framerate = 40 # FPS limit for camera v2 at full resolution is 15fps
        # you can set camera to different capture modes by uncommenting next line of code
        # but you must also make use_video_port = True in camera.capture() to change mode
        # some capture modes allow higher fps and binning (binning = higher SNR)
        
        camera.sensor_mode = 5
        image = np.empty((yRes * xRes * 3,), dtype=np.uint8)
        camera.capture(image, 'bgr', use_video_port = True) #not using video port forces the pi to use full resolution
        img1 = image.reshape((yRes, xRes , 3))
        # print (img1.size)
        # print (img1.shape)
        hsv1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
        

        sumH1 = 0
        sumS1 = 0
        sumV1 = 0

        for r in range (calib_row, calib_row + (calibrator_dim - 1)):
            for c in range (calib_col, calib_col + (calibrator_dim - 1)):
                sample_pix = hsv1[r,c]
                sumH1 = sample_pix[0] + sumH1 #hue running sum
                sumS1 = sample_pix[1] + sumS1 #saturation running sum
                sumV1 = sample_pix[2] + sumV1 #value running sum
        
        avgH1 = sumH1/((calibrator_dim ** 2)) #avg hue
        avgS1 = sumS1/((calibrator_dim ** 2)) #avg saturation
        avgV1 = sumV1/((calibrator_dim ** 2)) #avg value
        
        hsv_range = [avgH1 - 30, avgS1 - 40,  avgV1 - 80, avgH1 + 30, avgS1 + 40, avgV1 + 140]

        # Make sure low end threshold does not go below 0 
        if hsv_range[0] < 0.0:
            hsv_range[0] = 0.0
        
        if hsv_range[1] < 0.0:
            hsv_range[1] = 0.0
        
        if hsv_range[2] < 0.0:
            hsv_range[2] = 0.0

        # Make sure high end threshold does not go above limit

        if hsv_range[3] > 180.0:
            hsv_range[3] = 180.0
            
        if hsv_range[4] > 255.0:
            hsv_range[4] = 255.0

        if hsv_range[5] > 255.0:
            hsv_range[5] = 255.0
        #print(hsv_range)
    return hsv_range


    
def navigate(hsv_range):
    with picamera.PiCamera() as camera:
        camera.resolution = (xRes, yRes)
        camera.framerate = 40 # FPS limit for camera v2 at full resolution is 15fps
        # you can set camera to different capture modes by uncommenting next line of code
        # but you must also make use_video_port = True in camera.capture() to change mode
        # some capture modes allow higher fps and binning (binning = higher SNR)
        
        camera.sensor_mode = 5
        image = np.empty((yRes * xRes * 3,), dtype=np.uint8)
        camera.capture(image, 'bgr', use_video_port = True) #not using video port forces the pi to use full resolution/fov
        img1 = image.reshape((yRes, xRes , 3))

        # print (img1.size)
        # print (img1.shape)

        hsv1 = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
        
        cropRd = hsv1[yCropTop:yCropBot, 0:]
        mask1 = cv2.inRange(cropRd, (hsv_range[0],hsv_range[1], hsv_range[2]), (hsv_range[3],hsv_range[4], hsv_range[5]))


        # Course Correction Sampling/ Intersection Recognition using 64 10x10 samples from the mask
        # Running on the masks, where 0 is black and 255 is white

        
        #Initializing Values


        sample_sum = [0] * num_sample # Create a list (array) of num_sample size initialized to 0 for course correction
        left_intersect_sum = [0] * intersect_span #array initialized to 0 used for intersection detection
        right_intersect_sum = [0] * intersect_span #array initialized to 0 used for intersection detection
        output = Float32MultiArray()
        output.data = [0] * 3
        #output is a Float 32 array to be returned by the navigate() function
        #output[0] outputs course correction angle
        #output[1] outputs a 1.0 if there is a left intersection, 0.0 if there is none

        

        navi_running_sum = 0 # Running Sum for one sample in course correction
        left_running_sum = 0 # Running Sum for one sample on left intersection detection
        right_running_sum = 0 # Running Sum for one sample on left intersection detection
        left_sum = 0
        right_sum = 0

        aligned = True
        left_intersection = False
        right_intersection = False
        left_exist = 0.0
        right_exist = 0.0
        # Find the first 2 adjacent road samples from the left, the leftmost sample is the left edge of the road.
        # Find the last 2 adjacent road samples, the rightmost one is the right edge of the road
        left_edge_counter = 0
        right_edge_counter = 0
        left_edge_index = 0
        right_edge_index = 0
        left_edge_found = False
        right_edge_found = False
        edge_sum = 0
        
        

        # evenly divide the mask into num_sample regions from left to right, at the center of each region is a 10x10 processing area to check if road exists there
        # This is used only for course correction. Intersection Detection comes later and only occurs if car is aligned with the road (on good trajectory)
        # remember if you have a loop " for i in range (0,10): ", this loop will go from 0 to 9
        for i in range (0,num_sample): 
            for r in range (0, sample_dim):
                for c in range (left_col, left_col + sample_dim):
                    
                    navi_running_sum = mask1[(sample_dim - 1) + height_btwn_samples + r,c + i * left2right_col] + navi_running_sum #running sum of one sample for course correction
                    
                    
                    
            sample_sum[i] = navi_running_sum
            navi_running_sum = 0
            
            if (left_edge_found == False) and (sample_sum[i] > sample_threshold):
                left_edge_counter += 1 #increment road edge counter
                if left_edge_counter == edge_threshold:
                    left_edge_index = i-(edge_threshold - 1)
                    left_edge_found = True
                    left_edge_counter = 0
            
            else:
                left_edge_counter = 0
                    

        
        for i in reversed(range(0,num_sample)):
            if (right_edge_found == False) and (sample_sum[i] > sample_threshold):
                right_edge_counter += 1 #increment road edge counter
                        
                if right_edge_counter == edge_threshold:
                    right_edge_index = i+(edge_threshold - 1)
                    right_edge_found = True
                    right_edge_counter = 0
                    break
            else:
                right_edge_counter = 0
        #If no road is found, there is probably masking inaccuracy. Go straight until the road masking is in range again. 
        
        if (left_edge_found == False) or (right_edge_found == False):
            left_edge_index = (num_sample / 2) - 1;
            right_edge_index = num_sample / 2;

        # Turn off Intersection Recognition when large angles occurs
        # This is to prevent angled roads being recognized as intersection
        
        if left_edge_index > large_lean_left:
            aligned = False
        if right_edge_index < large_lean_right:
            aligned = False
        
        #If car is properly aligned to road, then check intersections
        if aligned == True:

            for i in range (0,intersect_span): 
                for r in range (0, sample_dim):
                    for c in range (left_col, left_col + sample_dim):
                        left_running_sum = mask1[r,c + i * left2right_col] + left_running_sum #running sum of one sample for intersection detection
                        right_running_sum = mask1[r,(num_sample - 1) - (c + i * left2right_col)] + right_running_sum #running sum of one sample for intersection detection
                
                left_intersect_sum[i] = left_running_sum
                right_intersect_sum[i] = right_running_sum
                left_running_sum = 0
                right_running_sum = 0
                
                if i < left_intersect_end - 1:
                    # summing up to check for left intersection
                    left_sum = left_intersect_sum[i] + left_sum
                    
                if i == left_intersect_end - 1:
                    # now check if left intersection exists 
                    if (left_sum > 1*sample_threshold * intersect_span): # if intersection is detected on the left side
                        left_intersection = True # then an intersection exists on the left
                
                
                if (i < num_sample - 1) and (i>= num_sample - intersect_span):
                    # summing up to check for left intersection
                    right_sum = right_intersect_sum[i] + right_sum
                    
                if i == num_sample - 1:
                    # now check if left intersection exists 
                    if (right_sum > 1*sample_threshold * intersect_span): # if road is detected in the outer eight regions on the right side
                        right_intersection = True # then an intersection exists on the right
        
        #If there is a intersection, limit course correction to only go for right intersections
        if (right_turn_only and left_intersection) == True:
            left_edge_index = (int) ((4/3) * intersect_span)

        if (left_turn_only and right_intersection) == True:
            right_edge_index = (int) (num_sample - (4/3) * intersect_span)
        
            
        
        #Angle Calculation
        edge_sum = left_edge_index + right_edge_index

        center_sample = (edge_sum + 1)/ 2
        
        xAngle = (center_sample * region_angle) - (xFov / 2) #left of center is negative angle, right of center is positive
        
        if left_intersection == True:
            left_exist = 1.0
        else:
            left_exist = 0.0
            
        if right_intersection == True:
            right_exist = 1.0
        else:
            right_exist = 0.0
            
        output.data = [xAngle, left_exist, right_exist]
                
    return output

def cam_talker(hsv_range):
    pub = rospy.Publisher('cam_topic_william', Float32MultiArray, queue_size=1)
    rospy.init_node('cam_test_node', anonymous = True)
    rate = rospy.Rate(10)
    navi_output = navigate(hsv_range)
    pub.publish(navi_output)
    rate.sleep()
    
    
    
    
if __name__ == '__main__':
   
    hsv_range = calibrate()
    while True:
        try:
            cam_talker(hsv_range)
        except KeyboardInterrupt:
            break
