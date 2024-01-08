import cv2 as cv
from sys import argv, exit
from typing import Sequence, List

shapeTemplate : cv.UMat
# shapeTemplate should be a single binary mat of a pixel on which...
# ...canny edge detection was applied before
image : cv.Mat = cv.imread(argv[0])

image = cv.cvtColor(image, cv.COLOR_RGB2GRAY)

edges : cv.UMat = None

cv.Canny(image, image, 50, 100, edges, false)

contours : Sequence[cv.UMat]
contours, _ = cv.findContours(edges, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

pixels : int = 0
boundingRects : List[cv.typing.Rect] = list()
for contour in contours:
    if (cv.matchShapes(contour, shapeTemplate, cv.CONTOURS_MATCH_I2, 0) < 0.5):
        pixels += 1
        boundingRects.append(cv.boundingRect(contour))

print(pixels)

for i in range(0, boundingRects.__len__()):
    rect : cv.typing.Rect = boundingRects[i]
    print(rect.__repr__())

# choose area of interest
# apply canny
# find contours
# matchShapes - contours
# if the shape matches well, it's a pixel -> get bounding rect
# else return