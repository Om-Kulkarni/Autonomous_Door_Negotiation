import numpy as np
import cv2

with open("image.txt", "r") as file:
    content = file.read()
content = content.split(',')

print(len(content))

content = [int(i) for i in content]

# reshape the content to 640x480x3
content = np.array(content).reshape((480, 640, 3))
print(content.shape)
cv2.imwrite("output_image.png", content)


