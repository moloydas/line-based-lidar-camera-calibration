# show_detected_lines.py
import numpy as np
import argparse
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import csv

def list_2_str(l):
    s = ''
    for idx,item in enumerate(l):
        if idx != 0:
            s += ','
        s += item
    s += '\n'

    return s

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='show detected lines')
    parser.add_argument('lines_file', help='give the npy pointcloud file')
    parser.add_argument('img', help='image file')
    args = parser.parse_args()

    img = cv2.imread(args.img)

    itr = 0
    file_writer = open('img_imp_lines_1.txt', 'w+')
    with open(args.lines_file) as csvfile:
        reader = csv.reader(csvfile, delimiter=' ')
        for rows in reader:
            
            temp = cv2.line(img.copy(), (int(float(rows[0])),int(float(rows[1]))), (int(float(rows[2])),int(float(rows[3]))), (0,0,255), 5)
            cv2.putText(temp, str(itr), (50, 50), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,255), 2, cv2.LINE_AA)
            cv2.imshow('image', temp)
            key = cv2.waitKey(0)
            itr += 1
            if key == ord('q'):
                break
            elif key == 32 or key == 13:
                print(rows[:7])
                file_writer.write(list_2_str(rows[:7]))

