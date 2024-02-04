import cv2
import numpy as np
import time
from rknnpool import rknnPoolExecutor
from func import myFunc

# decice tree for RK356x/RK3588
DEVICE_COMPATIBLE_NODE = '/proc/device-tree/compatible'

CLASSES = ("note", "robot")
INPUT_SIZE = 224

RK3588_RKNN_MODEL = '../models/note-model.rknn'

TPEs = 6

def print_outputs(boxes, classes, scores):
    
    if boxes is not None:
        for box, score, cl in zip(boxes, scores, classes):
            top, left, right, bottom = [int(_b) for _b in box]
            print("%s @ (%d %d %d %d) %f" % (CLASSES[cl], top, left, right, bottom, score))
    else:
        print("No notes detected")

if __name__ == '__main__':

    cam = cv2.VideoCapture(0)

    pool = rknnPoolExecutor(
        rknnModel=RK3588_RKNN_MODEL,
        TPEs=TPEs,
        func=myFunc)

    if (cam.isOpened()):
        for i in range(TPEs + 1):
            ret, frame = cam.read()
            if not ret:
                cam.release()
                del pool
                exit(-1)
            pool.put(frame)

    frames, loopTime, initTime = 0, time.time(), time.time()

    while (cam.isOpened()):
        frames += 1
        ret, frame = cam.read()

        if not ret:
            break

        pool.put(frame)
        data, flag = pool.get()

        if flag == False:
            break

        print_outputs(data[0], data[1], data[2])
        
        # exit condition
        if frames > 300:
            break

        # calculate average fps every 30 frames
        if frames % 30 == 0:
            print("FPS:\t", 30 / (time.time() - loopTime))
            loopTime = time.time()

    # Overall average fps
    print("Overall fps", frames / (time.time() - initTime))

    cam.release()
    pool.release()