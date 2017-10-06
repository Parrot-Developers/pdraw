#!/usr/bin/env python2

import cv2
import numpy as np
import os
import sys

# import pdraw_python with right path
try:
    l = os.environ['LD_LIBRARY_PATH']
    path = l.split(":")[1] + "/python"
    sys.path.insert(0, path)
    import pdraw_python as pdraw
except:
    print "launch me with native_wrapper please"
    sys.exit(-1)


ipSrc = "192.168.42.1"
portStreamSrc = 55004
portCtrlSrc = 55005

# set 2 next ports from sdk log
portStreamDst = 55004
portCtrlDst = 55005

mypdraw = pdraw.createPdraw()
mypdraw.setSelfSerialNumber("00000000")
mypdraw.open(ipSrc, "", portStreamSrc, portCtrlSrc, portStreamDst, portCtrlDst, 0)

pdrawProd = mypdraw.addVideoFrameProducer(0)
mypdraw.start()

nb = 1000
frame = None

n = 0
while True:
    # block 100ms max for frame
    frame = mypdraw.getProducerLastFrame(pdrawProd, 100000)
    if frame is not None and frame.width:
        print "frame", n
        cv2.imshow("frame", frame.plane(0))
        cv2.waitKey(2)
        n += 1

mypdraw.stop()
mypdraw.removeVideoFrameProducer(pdrawProd)
