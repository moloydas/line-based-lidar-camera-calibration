## Line Based Lidar Camera Calibration
We propose a lidar and camera calibration method which uses Resection loss to calculate the rotation and translation matrices.  

![Resection Error](https://drive.google.com/open?id=1WWuaMlYl_rD1BXUh1Utm3mnFUZhKcOJ6)
In the image above, "AB" is the 3D line segment, " A'B' " is the projection of AB line on the image plane and "ab" is the 2D line segment in the image.

The cross product of lines oa and ob forms a Normal vector "N". The 3D line AB is rotated by R and Translated by T which then forms vector "M" in camera coordinate system. The Resection loss is defined as the dot product of the N and M. 

$$
\widehat{N} = oa \otimes ab \\ \widehat{M} = R \times AB + T \\ E_{j} = \widehat{N} \circledcirc \widehat{M} \\Total loss = \sum_{j=1}^{N}E_{j}
$$
where N is the number of correspondences available.

