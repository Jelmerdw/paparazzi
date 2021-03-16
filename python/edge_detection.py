#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  3 19:32:22 2021

@author: jelmer
"""

import numpy as np
import cv2 as cv
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy import ndimage
import os.path

#Load images:
if os.path.isfile('images.npy'):
    images = np.load('images.npy')
else:
    image_list = os.listdir('images')
    images = []
    
    for i in image_list:
        path = str('images/' + i)
        img = mpimg.imread(path)
        img_rotated = ndimage.rotate(img, 90)
        images.append(img_rotated)

    images = np.asarray(images)
    np.save('images.npy', images)
    print("Images saved")

#Remove 50 first images:
images = images[50:]

#Setting:
vertical_kernel = cv.getStructuringElement(cv.MORPH_RECT, (1, 4))

rho = 1 # distance resolution in pixels of the Hough grid
theta = np.pi  # angular resolution in radians of the Hough grid
threshold = 15  # minimum number of votes (intersections in Hough grid cell)
min_line_length = 2  # minimum number of pixels making up a line
max_line_gap = 20  # maximum gap in pixels between connectable line segments

#Create figure:
plt.clf()
plt.figure(1)

#Do for all image:
for i in range(len(images)):
    points = []
    images[i] = cv.addWeighted(images[i], 1, images[i], 0, 0)
    
    # #Contour
    # imgray = cv.cvtColor(images[i], cv.COLOR_BGR2GRAY)
    # ret, thresh = cv.threshold(imgray, 90, 255, 0)
    # contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # img_contour = cv.drawContours(images[i].copy(), contours, -1, (0,255,0), 3)
    
    #Edges:
    img_edges = cv.Canny(images[i], 50, 100)
    
    #Vertical:
    img_vertical = cv.morphologyEx(img_edges, cv.MORPH_OPEN, vertical_kernel, iterations=1)
    
    #Lines:
    lines = cv.HoughLinesP(img_vertical, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
    
    #Create lines image and calculate center of line:
    img_lines = np.copy(images[i]) * 0
    if type(lines) != type(None):
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv.line(img_lines,(x1,y1),(x2,y2),(0,0,255),2)
                points.append([(x1+x2)/2,(y1+y2)/2])
                
    img_lines = cv.addWeighted(images[i], 1, img_lines, 1, 0)
    
    #Group points to objects using treshold:
    points = sorted(points)
    centers = []
    x_center = -1
    y_center = -1
    N = 0
    for p in points:
        if x_center == -1 or p[0] - x_center < 50:
            x_center = (x_center * N + p[0])/(N+1)
            y_center = (y_center * N + p[1])/(N+1)
            N += 1
        else:
                centers.append([x_center, y_center])
                x_center = -1
                y_center = -1
                N = 0
    if x_center != -1:
        centers.append([x_center, y_center])
    
    #Plot images:
    plt.clf()
    
    plt.subplot(221)
    plt.text(0, -8, "Original image", fontsize=12)
    plt.imshow(images[i])
    
    plt.subplot(222)
    plt.text(0, -8, "Edge detection", fontsize=12)
    plt.imshow(img_edges)
    
    plt.subplot(223)
    plt.text(0, -8, "Vertical lines", fontsize=12)
    plt.imshow(img_vertical)
    
    plt.subplot(224)
    plt.text(0, -8, "Filtered lines and object mark", fontsize=12)
    for c in centers:
        plt.plot(c[0], c[1], 'go')
    plt.imshow(img_lines)
    
    name = str("result/frame" + str(i) + ".png")
    #plt.savefig(name, dpi=300)
    plt.pause(0.0001)