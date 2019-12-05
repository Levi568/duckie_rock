#!/usr/bin/env python
import sys
import cv2
import numpy as np

def get_lines(original_image, filtered_image):
    # do our hough transform on the white image
    # resolution: 1 pixel radius, 1 degree rotational
    r_res = 1
    theta_res = np.pi/180
    # threshold: number of intersections to define a line
    threshold = 6
    # min_length: minimum number of points to form a line
    min_length = 3
    # max_gap: maximum gap between two points to be considered a line
    max_gap = 4
    lines = cv2.HoughLinesP(filtered_image, r_res, theta_res, threshold,np.empty(1), min_length, max_gap)
    
    output = np.copy(original_image)
    if lines is not None:
        # grab the first line
        for i in range(len(lines)):
            print(lines[i])
            l = lines[i][0]
            cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,0,255), 3, cv2.LINE_AA)
    return output

def region_of_interest(img, vertices):
    # Define a blank matrix that matches the image height/width.
    mask = np.zeros_like(img)
    # Retrieve the number of color channels of the image.
    channel_count = img.shape[2]
    # Create a match color with the same color channel counts.
    match_mask_color = (255,) * channel_count
      
    # Fill inside the polygon
    cv2.fillPoly(mask, vertices, match_mask_color)

    # Returning the image only where mask pixels match
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def lane_filter(image):

    #imshape = image.shape
    
    #lower_left = [imshape[1]/20, imshape[0]]
    #lower_right = [imshape[1] - imshape[1]/25, imshape[0]]
    #top_left = [imshape[1]/2 - imshape[1]/3, imshape[0]/2+imshape[0]/15]
    #top_right = [imshape[1]/2 + imshape[1]/8, imshape[0]/2+imshape[0]/20]
    
    #vertices = [np.array([lower_left, top_left,top_right, lower_right], dtype=np.int32)]
    #roi_image = region_of_interest(image, vertices)
    
    # The incoming image is BGR format, convert it to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Filter for only white pixels. Experiment with values as needed
    white_filter = cv2.inRange(hsv, (0,0,150), (255,60,255))
    ##cv2.imshow("White Filter", white_filter)
    cv2.imwrite("white_filter.png", white_filter)
    
    # Filter for only yellow pixels. Experiment with values as needed
    yellow_filter = cv2.inRange(hsv, (20,110,120), (45,255,255))
    cv2.imshow("Yellow Filter", yellow_filter)
    cv2.imwrite("yellow_filter.png", yellow_filter)
    
    # Create a kernel to dilate the image.
    # Experiment with the numbers in parentheses (optional)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    
    # Dilate both the white and yellow images.
    # No need to experiment here.
    #white_erode = cv2.erode(white_filter, kernel,iterations=1)
    white_dilate = cv2.dilate(white_filter, kernel, iterations=1)
    
    cv2.imshow("Dilate White", white_dilate)
    cv2.imwrite("white_dilate.png", white_dilate)
    
    #yellow_erode = cv2.erode(yellow_filter, kernel,iterations=5)
    yellow_dilate = cv2.dilate(yellow_filter, kernel,iterations=1)
    cv2.imshow("Dilate Yellow", yellow_dilate)
    cv2.imwrite("yellow_dilate.png", yellow_dilate)
    
    # Perform edge detection on the original image.
    # Experiment with the first two numbers. Aperture size experimentation optional
    edges = cv2.Canny(image, 100, 200, apertureSize=3) # find the edges
    cv2.imshow("Edges", edges)
    cv2.imwrite("edges.png", edges)
    
    # Use the edges to refine the lines in both white and yellow images
    # No need to experiment here
    white_edges = cv2.bitwise_and(white_dilate, edges)
    cv2.imshow("White Edges", white_edges)
    cv2.imwrite("white_edges.png", white_edges)
    yellow_edges = cv2.bitwise_and(yellow_dilate, edges)
    cv2.imshow("Yellow Edges", yellow_edges)
    cv2.imwrite("yellow_edges.png", yellow_edges)
    
    white_output = get_lines(image, white_edges)
    yellow_output = get_lines(image, yellow_edges)
    
    cv2.imshow("White Output", white_output)
    cv2.imwrite("white_output.png", white_output)
    cv2.imshow("Yellow Output", yellow_output)
    cv2.imwrite("yellow_output.png", yellow_output)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: %s image_filename.png" % sys.argv[0])
        exit()
        
    image_filename = sys.argv[1]
    image = cv2.imread(image_filename)
    lane_filter(image)
    print("Ctrl + C to quit ")
    
    while True:
        # Wait for key press to close images
        if cv2.waitKey(1) & 0xFF == 27:
            print("quit windows")
            break
