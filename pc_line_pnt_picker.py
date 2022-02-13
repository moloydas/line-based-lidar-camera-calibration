# project_3d_pts_img.py
import numpy as np
import argparse
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import random

f = 692.653256; cx = 629.321381; cy = 330.685425
p = np.array([ [692.653256, 0.000000, 629.321381, 0.000000],
                [0.000000, 692.653256, 330.685425, 0.000000],
                [0.000000, 0.000000, 1.000000, 0.000000] ])

K = np.array([ [f, 0.0, cx],
                [0.0, f, cy],
                [0.0, 0.0, 1.0] ])

points_captured = []
points_3d_captured = []

def find_closest_pts(pts, pt, radius):
    dist = np.sqrt(np.sum(np.square(pts - pt), axis=0))
    return pts[:,dist < radius]

def click_event(event, x, y, flags, params):
    global points_captured
    global points_3d_captured
    if event == cv2.EVENT_LBUTTONDOWN:
        closest_pts = find_closest_pts(params[0], np.array([x,y, 1]).reshape(3,1), 20)

        for x,y,_ in closest_pts.T:
            points_captured.append([x,y])
            loc = np.all(params[0] == np.array([x,y,1], dtype=np.float).reshape(3,1), axis=0)
            points_3d_captured.append(params[2][loc, :])

            cv2.circle(img, (int(x), int(y)), 5, params[1], -1)
        cv2.imshow('image', img)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='extract the first pointcloud from the bag')
    parser.add_argument('pts_file', help='give the npy pointcloud file')
    args = parser.parse_args()

    pts = np.load(args.pts_file)

    R = np.array([  [0,-1,0],
                    [0,0,-1],
                    [1,0,0]])

    new_pt = K @ R @ pts.T
    new_pt /= new_pt[2]

    img = cv2.imread('raw_data/image.png')

    for x,y,_ in new_pt.T:
        if x > 0 and y>0:
            cv2.circle(img, (int(x), int(y)), 5, (0,255,255), -1)

    i = 0
    line_pts = []

    lines_writer = open('picked_edge_lines.txt', 'w+')
    lines_3d_writer = open('picked_3d_edge_lines.txt', 'w+')

    while(i < 20):
        cv2.imshow("image", img)
        r_random = int(random.random()*255)
        g_random = int(random.random()*255)
        b_random = int(random.random()*255)

        cv2.setMouseCallback('image', click_event, [new_pt, (b_random, g_random, r_random), pts])
        key = cv2.waitKey(0)
        i += 1
        if key == ord('q'):
            break
        elif key == 32 or key == 13:
            line_pts.append(points_captured.copy())

            if len(points_captured) == 0:
                continue

            ## get the two extremities in the 3d line
            min_x = np.argmin(np.array(points_captured)[:,0])
            max_x = np.argmax(np.array(points_captured)[:,0])

            extreme_1 = points_3d_captured[min_x].flatten()
            extreme_2 = points_3d_captured[max_x].flatten()

            lines_3d_writer.write(  str(extreme_1[0]) + ',' + str(extreme_1[1]) + ',' + str(extreme_1[2]) + ',' + 
                                    str(extreme_2[0]) + ',' + str(extreme_2[1]) + ',' + str(extreme_2[2]) + '\n')

            ## fit a line and draw it on the image for reference 
            line_points = np.array(line_pts[-1])
            par = np.polyfit(line_points[:,0], line_points[:,1], 1, full=True)
            slope=par[0][0]
            intercept=par[0][1]
            xl = [min(line_points[:,0]), max(line_points[:,0])]
            yl = [slope*xx + intercept  for xx in xl]
            lines_writer.write(str(xl[0]) + ',' + str(yl[0]) + ',' + str(xl[1]) + ',' + str(yl[1]) + ',' + str(slope) + ',' + str(intercept) + '\n')
            cv2.line(img, (int(xl[0]),int(yl[0])), (int(xl[1]),int(yl[1])), (0,0,255), 5)

            points_captured.clear()
            points_3d_captured.clear()

    lines_writer.close()
    lines_3d_writer.close()
    cv2.destroyAllWindows()
