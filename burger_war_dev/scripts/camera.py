#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

def processImage(frame, color):
    # color binarization
    if color == "yellow":
        mask, masked_img = detectYellowColor(frame)
    elif color == "green":
        mask, masked_img = detectGreenColor(frame)
    elif color == "blue":
        mask, masked_img = detectBlueColor(frame)
    else:
        return -1, None

    out_img = masked_img
    num_labels, label_image, stats, center = cv2.connectedComponentsWithStats(mask)

    num_labels = num_labels - 1
    stats = np.delete(stats, 0, 0)
    center = np.delete(center, 0, 0)

    if num_labels == 0:
        return False, out_img
    
    for index in range(num_labels):
        x = stats[index][0]
        y = stats[index][1]
        w = stats[index][2]
        h = stats[index][3]
        s = stats[index][4]
        mx = int(center[index][0])
        my = int(center[index][1])
        #print("(x,y)=%d,%d (w,h)=%d,%d s=%d (mx,my)=%d,%d"%(x, y, w, h, s, mx, my) )

        cv2.rectangle(out_img, (x, y), (x+w, y+h), (255, 0, 255))

    return True, out_img

def detectYellowColor(frame):
    # BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Range of Green
    hsv_min = np.array([30, 64, 0])
    hsv_max = np.array([60,255,255])
    # Mask of Green
    mask = cv2.inRange(hsv, hsv_min, hsv_max)
    # Masking
    masked_img = cv2.bitwise_and(frame, frame, mask=mask)

    return mask, masked_img

def detectGreenColor(frame):
    # BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Range of Green
    hsv_min = np.array([60, 64, 0])
    hsv_max = np.array([90,255,255])
    # Mask of Green
    mask = cv2.inRange(hsv, hsv_min, hsv_max)
    # Masking
    masked_img = cv2.bitwise_and(frame, frame, mask=mask)

    return mask, masked_img

def detectBlueColor(frame):
    # BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Range of Green
    hsv_min = np.array([90, 64, 0])
    hsv_max = np.array([150,255,255])
    # Mask of Green
    mask = cv2.inRange(hsv, hsv_min, hsv_max)
    # Masking
    masked_img = cv2.bitwise_and(frame, frame, mask=mask)

    return mask, masked_img

def showImage(img):
    cv2.imshow("Image window", img)
    cv2.waitKey(1)
