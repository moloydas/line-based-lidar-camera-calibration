#crop_pointcloud.py
import numpy as np
import argparse
import open3d as o3d
import open3d

parser = argparse.ArgumentParser(description='extract the first pointcloud from the bag')
parser.add_argument('pointcloud_file', help='give the pcd pointcloud file')
args = parser.parse_args()

pcd = o3d.io.read_point_cloud(args.pointcloud_file)

center = np.array([10,0,0])
R = np.eye(3)
extent = np.array([20,5,10])
bbox = o3d.geometry.OrientedBoundingBox(center,R,extent)
# print(np.asarray(bbox.get_box_points()))
pcd = pcd.crop(bbox)

axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.6, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcd, axes])

# o3d.io.write_point_cloud("test_pointcloud_cropped.pcd", pcd)
