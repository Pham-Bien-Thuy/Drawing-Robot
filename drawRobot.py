import os
import time
import sys
from PIL import Image, ImageFilter
import numpy as np
import cv2 as cv
import rospy
from uavlab411.msg import drawing_point_msg
import csv

sys.setrecursionlimit(2000)  

coordinates = []

pic_path = 'cat.png'  # Image Path
alpha = 0.05  
winSize = 21  
draw_delay = 0.001  
jd = 1 
gl = 200 

def busy_sleep():  
    start = time.time()
    while (time.time() < start + draw_delay):
        pass


def mouse_up_judge(xx, yy):  
    global drawing_point
    position_x, position_y = drawing_point.x, drawing_point.y
    if position_x - jd != xx and position_x + jd != xx and position_y + jd != yy and position_y - jd != yy:
        drawing_point.z = 52
        coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
        return True
    else:
        return False


def dfs(img, x, y, first, count=0):
    global drawing_point, pub
    if count >= 1900:  
        return [x, y]
    count += 1

    if first == True:
        drawing_point.x = y
        drawing_point.y = x
        drawing_point.z = 55
        coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))

    if y + jd < cols and x - jd >= 0:
        if img[x - jd, y + jd] == 0:
            mouse_judge = mouse_up_judge(y + jd, x - jd)
            drawing_point.x = y + jd
            drawing_point.y = x - jd
            coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            if mouse_judge == True:
                drawing_point.z = 55
                coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))

            img[x - jd, y + jd] = 1
            busy_sleep()
            dfs(img, x - jd, y + jd, False, count)

    if y + jd < cols and x + jd < rows:
        if img[x + jd, y + jd] == 0:
            mouse_judge = mouse_up_judge(y + jd, x + jd)
            drawing_point.x = y + jd
            drawing_point.y = x + jd
            coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            if mouse_judge == True:
                drawing_point.z = 55
                coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))

            img[x + jd, y + jd] = 1
            busy_sleep()
            dfs(img, x + jd, y + jd, False, count)

    if y - jd >= 0 and x - jd >= 0:
        if img[x - jd, y - jd] == 0:
            mouse_judge = mouse_up_judge(y - jd, x - jd)
            drawing_point.x = y - jd
            drawing_point.y = x - jd
            coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            if mouse_judge == True:
                drawing_point.z = 55
                coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            img[x - jd, y - jd] = 1
            busy_sleep()
            dfs(img, x - jd, y - jd, False, count)

    if y - jd >= 0 and x + jd < rows:
        if img[x + jd, y - jd] == 0:
            mouse_judge = mouse_up_judge(y - jd, x + jd)
            drawing_point.x = y - jd
            drawing_point.y = x + jd
            coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            if mouse_judge == True:
                drawing_point.z = 55
                coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            img[x + jd, y - jd] = 1
            busy_sleep()
            dfs(img, x + jd, y - jd, False, count)

    if y + jd < cols:
        if img[x, y + jd] == 0:
            mouse_judge = mouse_up_judge(y + jd, x)
            drawing_point.x = y + jd
            drawing_point.y = x
            coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z)) 
            if mouse_judge == True:
                drawing_point.z = 55
                coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))

            img[x, y + jd] = 1
            busy_sleep()
            dfs(img, x, y + jd, False, count)

    if y - jd >= 0:
        if img[x, y - jd] == 0:
            mouse_judge = mouse_up_judge(y - jd, x)
            drawing_point.x = y - jd
            drawing_point.y = x
            coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z)) 
            if mouse_judge == True:
                drawing_point.z = 55
                coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            img[x, y - jd] = 1
            busy_sleep()
            dfs(img, x, y - jd, False, count)

    if x - jd >= 0:
        if img[x - jd, y] == 0:
            mouse_judge = mouse_up_judge(y, x - jd)
            drawing_point.x = y 
            drawing_point.y = x - jd
            coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            if mouse_judge == True:
                drawing_point.z = 55
                coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            img[x - jd, y] = 1
            busy_sleep()
            dfs(img, x - jd, y, False, count)

    if x + jd < rows:
        if img[x + jd, y] == 0:
            mouse_judge = mouse_up_judge(y, x + jd)
            drawing_point.x = y 
            drawing_point.y = x + jd
            coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            if mouse_judge == True:
                drawing_point.z = 55
                coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            img[x + jd, y] = 1
            busy_sleep()
            dfs(img, x + jd, y, False, count)

    return None


if __name__ == '__main__':
    global drawing_point, pub
    pub = rospy.Publisher("uavlab411/drawing_point", drawing_point_msg, queue_size=10)
    rospy.init_node("taker", anonymous=True)
    rate = rospy.Rate(5)
    drawing_point = drawing_point_msg()

    img = cv.imread(pic_path, 0)  # img path
    img_blur = cv.GaussianBlur(img, (winSize, winSize), 5)
    img = np.uint8(img > (1 - alpha) * img_blur) * 255
    cv.imwrite("temp_img.jpg", img)
    img = Image.open('temp_img.jpg')

    width = img.size[0]
    height = img.size[1]

    img = img.convert('L')
    img = np.array(img)
    rows, cols = img.shape
    for i in range(rows):
        for j in range(cols):
            if img[i, j] <= gl:
                img[i, j] = 0
            else:
                img[i, j] = 1

    img = Image.fromarray(img)
    img = img.filter(ImageFilter.SMOOTH)
    img = np.array(img)

    input("start?")
    print("Waiting for process image ...")

    count_i = 0
    while count_i < rows:
        count_j = 0
        while count_j < cols:
            if count_j + jd < cols:
                if img[count_i, count_j] == 0:
                    dfs_result = dfs(img, count_i, count_j, True, 0)
                    while dfs_result != None:
                        drawing_point.z = 52
                        coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
                        dfs_result = dfs(img, dfs_result[0], dfs_result[1], True, 0)
                    drawing_point.z = 52
                    coordinates.append((drawing_point.x, drawing_point.y, drawing_point.z))
            count_j += jd
        count_i += jd

    #csv_file_path = "drawRe.csv"
    #with open(csv_file_path, mode='w', newline='') as file:
    #    writer = csv.writer(file)
    #    for coord in coordinates:
    #        writer.writerow([coord[0], coord[1], coord[2]])
    for coor in coordinates:
        drawing_point.x = coor[0]
        drawing_point.y = coor[1]
        drawing_point.z = coor[2]
        print(coor[0], ' ', coor[1], ' ', coor[2])
        pub.publish(drawing_point)
        rate.sleep()
    #print(img.shape, ' ', width, ' ', height)
    os.remove('temp_img.jpg')
