#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

def processImage(frame):
    mask, masked_img = detectYellowColor(frame)
    return masked_img

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
