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
kernel = np.ones((5,5),np.uint8)

#Create figure:
plt.clf()
plt.figure(1)


#Do for all image:
for i in range(len(images)):
    image = cv.addWeighted(images[i], 1, images[i], 0, 0)
    image = cv.resize(image, dsize=(130, 60))
    #image = image[:][50:150]
    
    
    #Edges:
    img_edges = cv.Canny(image, 50, 100)    #50, 100
    img_edges[0, :] = 255
    img_edges[-1, :] = 255
    img_edges[:, 0] = 255
    img_edges[:, -1] = 255
    
    dilation = img_edges
    not_ended = True
    while not_ended:
        copy = dilation
        dilation = cv.dilate(dilation,kernel,iterations = 1)
        zeros = np.where(dilation == 0)
        if len(zeros[0]) == 0:
            dilation = copy
            not_ended = False
        
        
    zeros = np.where(dilation == 0)
    x_values = np.sort(zeros[1])
    
    goal = 0
    voters = 0
    
    priv = x_values[0]
    avg = 0
    N = 0
    for x in x_values:
        if x-priv <= 1:
            avg = (avg * N + x)/(N+1)
            N +=1
            priv = x
        else:
            if N > voters:
                goal = avg
                voters = N
            
            avg = x
            N = 1
            priv = x
    
    if N > voters:
         goal = avg
    
    #Plot images:
    plt.clf()
    
    plt.subplot(221)
    plt.text(0, -8, "Original image", fontsize=12)
    plt.imshow(images[i])
    
    plt.subplot(222)
    plt.text(0, -8, "Edge detection", fontsize=12)
    plt.imshow(img_edges, cmap="gray")
    
    plt.subplot(223)
    plt.text(0, -8, "Dilation", fontsize=12)
    plt.imshow(dilation, cmap="gray")
    
    plt.subplot(224)
    plt.text(0, -8, "Free space", fontsize=12)
    plt.imshow(images[i])
    plt.plot(goal*4, int(images[i].shape[0]/2), 'ro')
    
    name = str("result/frame" + str(i) + ".png")
    plt.savefig(name, dpi=300)
    plt.pause(0.0001)