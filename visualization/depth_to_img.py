
# from scipy.misc import toimage
import numpy as np
import cv2


def load_depth(idx):
    path = "../bin/to/depth"+str(idx)+".txt"
    
    with open(path) as f:
        text = f.read()
        
    content = text.split("\n")
    
    data = content[1].split(" ")

    width, height = content[0].split(" ")

    print("width" + width)
    print("height" + height)

    print(len(data))

    depth = np.array(data).reshape((int(height),int(width))).astype(float)
    max_val = (np.max(depth))
    depth = depth / max_val * 255

    print(np.max(depth))

    return depth
    
    # plt.imshow(depth)

iterations = 80

if __name__=='__main__':
    import time
    for i in range(iterations):
        depth = load_depth(i)
        # toimage(depth).show()
        cv2.imshow('image', depth)

        cv2.imwrite('im'+str(i)+".png", depth)
