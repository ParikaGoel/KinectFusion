
# from scipy.misc import toimage
import numpy as np
import cv2


def load_depth():
    path = "../bin/depth.txt"
    
    with open(path) as f:
        text = f.read()
        
    content = text.split("\n")
    data = content[1].split(",")[:-1]

    width, height = content[0].split(",")

    print(len(data))

    depth = np.array(data).reshape((int(height),int(width))).astype(float)

    return depth
    
    # plt.imshow(depth)

if __name__=='__main__':
    import time
    while True:
        time.sleep(5)
        depth = load_depth()

        # toimage(depth).show()
        cv2.imshow('image', depth)
        cv2.waitKey(1)
