#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 22 14:10:50 2021

@author: jelmer
"""

import numpy as np
import cv2
import imutils

import os
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;file,rtp,udp"

cap = cv2.VideoCapture('sw/tools/rtp_viewer/rtp_5000.sdp')

while(True):
    # Capture frame-by-frame and data:
    ret, frame = cap.read()
    data = open("data.txt", "r")
    x_value = int(data.read())

    # Rotate image:
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    rotated = imutils.rotate_bound(image, -90)
    
    #Add goal marker:
    result = cv2.circle(rotated, (x_value, int(image.shape[1]/2)), 5, color=(0, 0, 255), thickness=-1)

    # Display the resulting frame
    cv2.imshow('frame',result)
    print("Target = ", x_value)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()