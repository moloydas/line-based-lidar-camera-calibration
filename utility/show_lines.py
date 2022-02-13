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

    with open(args.lines_file) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for rows in reader:
            cv2.line(img, (int(float(rows[0])),int(float(rows[1]))), (int(float(rows[2])),int(float(rows[3]))), (0,0,255), 5)

        cv2.imshow('image', img)
        cv2.waitKey(0)
