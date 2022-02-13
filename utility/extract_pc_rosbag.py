import rosbag
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import argparse
import open3d as o3d
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='extract the first pointcloud from the bag')
parser.add_argument('bag_file', help='give the name of the bag file')
args = parser.parse_args()

bag = rosbag.Bag(args.bag_file, "r")

bridge = CvBridge()

lid_cap = False
img_cap = False

for topic, msg, t in bag.read_messages(topics=['/velodyne_points', '/zed/zed_node/right/image_rect_color']):
    if topic == '/velodyne_points':
        pc = pc2.read_points_list(msg, skip_nans=True, field_names=None)
        pc = np.array(pc, dtype=np.float32)

        # np.save('test.npy', pc)

        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(pc)

        # o3d.visualization.draw_geometries([pcd])
        # o3d.io.write_point_cloud("test.pcd", pcd)

        print("lidar cap time: " + str(msg.header.stamp))
        lid_cap = True
        if img_cap and lid_cap:
            break

    if topic == '/zed/zed_node/right/image_rect_color':
        print("img cap time: " + str(msg.header.stamp))

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        plt.imsave('image.png', cv_image)

        img_cap = True
        if img_cap and lid_cap:
            break
