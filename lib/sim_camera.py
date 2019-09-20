#!/usr/bin/env python

import glob

def get_images_from_directory(dir):
    filenames = glob.glob("%s/cam.*.jpg" %dir)
    filenames.sort()
    images = []
    for f in filenames:
        time = int(f[f.find("cam.")+4 : f.find(".jpg")-12])
        #print("Time: %.1f" %time)
        images += [[time, f]]
    return images

def find_image(time, images):
    last_image = None
    if (len(images) == 0): return None
    for image in images:
        dt = image[0] - time
        if (dt >= 0):
            if (last_image == None or dt < (time - last_image[0])):
                return image[1]
            else:
                return last_image[1]
        last_image = image
    return images[-1][1]

#import sys
#images = get_images_from_directory("Photos")
#while True:
#    time = int(sys.stdin.readline())
#    print(find_image(time, images))
