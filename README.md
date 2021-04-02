## Line Based Lidar Camera Calibration
We propose a lidar and camera calibration method which uses Resection loss to calculate the rotation and translation matrices.  

![diagram](https://drive.google.com/uc?export=view&id=1WWuaMlYl_rD1BXUh1Utm3mnFUZhKcOJ6)

![representation](https://drive.google.com/file/d/1FuLiLvGS6lB9LSbDOXE--23f4Wl1foXn/view?usp=sharing)

![Resection_loss diagram](https://user-images.githubusercontent.com/20353960/113435378-4694cd80-9400-11eb-892b-904641794cd8.png)

In the image above, "AB" is the 3D line segment, " A'B' " is the projection of AB line on the image plane and "ab" is the 2D line segment in the image.

The cross product of lines oa and ob forms a Normal vector "N". The 3D line AB is rotated by R and Translated by T which then forms vector "M" in camera coordinate system. The Resection loss is defined as the dot product of the N and M. 

![](https://drive.google.com/uc?export=view&id=1LvHcWJZlXorP54SF5at6lRwebdqZF0JY)

where N is the number of correspondences available.
