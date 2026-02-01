import heapq
import sys
import math
from PIL import Image

# =====IMAGE HELPERS=====
# Aside: Why do this when the functions are all just one liners?
# Consider switching to a different image library (opencv), representation, etc.
# This cleans up the code a bit by leaving the logic of the search algorithms independent of the specific image package.
# If we were to change, we would just need to update logic here once

def SetRGB(image, pixel_coordinate, color):
    image.putpixel(pixel_coordinate, color)

def GetRGB(image, pixel_coordinate):
    return image.getpixel(pixel_coordinate)

def SaveImage(image, filename, extension):
    image.save(filename, extension)

def OpenImage(filename):
    return  Image.open(filename)

if __name__ == '__main__':
    FileArg = sys.argv[1] #The name of the image file to open
    WeightArg = sys.argv[2] #The name of the weights .txt file to open
    MovementArg = sys.argv[3] #The type of movement the agent can take (Four versus Eight) directions
    Start = (int(sys.argv[4]), int(sys.argv[5]))
    Destination = (int(sys.argv[6]), int(sys.argv[7]))

    InputImage = FileArg+".png"

    TerrainImage = OpenImage(InputImage)
    WIDTH, HEIGHT = TerrainImage.width,TerrainImage.height

    