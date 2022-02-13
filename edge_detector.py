#edge_detector.py
import numpy as np
import argparse
import open3d as o3d
import open3d
import math

# config parameters
N_SCAN = 16
Horizon_SCAN = 1800
ang_res_x = 0.2
ang_res_y = 2.0
ang_bottom = 15.0+0.1
sample_set = 10
edgeThreshold = 0.001
edgeFeatureNum = 2

# create range image
def create_range_image(pc, crop_hori_ang_range=90):
    range_image = np.zeros((N_SCAN, Horizon_SCAN, 6))
    for pt in pc:
        horizonAngle = math.atan2(pt[1], pt[0]) * 180 / math.pi
        dist = np.sqrt(np.square(pt[0]) + np.square(pt[1]) + np.square(pt[2]) )
        if(horizonAngle < crop_hori_ang_range and horizonAngle > -crop_hori_ang_range):
            columnIdx = round(-1*horizonAngle/ang_res_x) + 900
            range_image[int(pt[4]),int(columnIdx),:5] = pt
            range_image[int(pt[4]),int(columnIdx),5] = dist
    return range_image

def range_img_2_pc(img):
    pc = []
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            if img[i,j,5] > 0.1:
                pc.append(img[i,j])
    return np.array(pc)

def calculate_corners(img):
    edge_img = np.zeros(img.shape)

    for i in range(img.shape[0]):
        corners = []
        for j in range(int(sample_set/2),img.shape[1]-int(sample_set/2)):
            if img[i,j,5] < 0.0001:
                continue
            s = img[i, (j-int(sample_set/2)):(j+int(sample_set/2)+1), 5]
            dist_set = s-img[i,j,5]
            sum_set = np.sum(dist_set)
            corner_score = (sum_set)/(img[i,j,5]*sample_set)

            corners.append(corner_score)

            if corner_score > edgeThreshold:
                edge_img[i,j,:] = img[i,j,:]

        # print(np.max(corners))

    return edge_img


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='extract the first pointcloud from the bag')
    parser.add_argument('pointcloud_file', help='give the npy pointcloud file')
    args = parser.parse_args()

    pc = np.load(args.pointcloud_file)

    #####################################
    ## convert pc to range image
    #####################################
    range_img = create_range_image(pc, crop_hori_ang_range=50)
    cropped_range_img = np.zeros((N_SCAN, 500, 6))
    cropped_range_img = range_img[:,650:1150,:]

    #####################################
    ## convert range image to pc
    #####################################
    # new_pc = range_img_2_pc(range_img)
    new_pc = range_img_2_pc(cropped_range_img)
    edge_img = calculate_corners(cropped_range_img)
    edge_pc = range_img_2_pc(edge_img)

    #####################################
    ## convert pc to open3d
    #####################################
    pc_viz = new_pc[:, :3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_viz)
    pcd.paint_uniform_color(np.array([1,1,0]))

    edges = o3d.geometry.PointCloud()
    edges.points = o3d.utility.Vector3dVector(edge_pc[:,:3])
    edges.paint_uniform_color(np.array([0,1,1]))

    edge_points = np.asarray(edges.points)
    np.save('edge_points.npy', edge_points)

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6, origin=[0, 0, 0])

    o3d.visualization.draw_geometries([edges, pcd , axes])
