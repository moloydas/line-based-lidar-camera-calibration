## Line Based Lidar Camera Calibration
We propose a lidar and camera calibration method which uses Resection loss to calculate the rotation and translation matrices.  

![diagram](https://drive.google.com/uc?export=view&id=1WWuaMlYl_rD1BXUh1Utm3mnFUZhKcOJ6)

In the image above, "AB" is the 3D line segment, " A'B' " is the projection of AB line on the image plane and "ab" is the 2D line segment in the image.

The cross product of lines oa and ob forms a Normal vector "N". The 3D line AB is rotated by R and Translated by T which then forms vector "M" in camera coordinate system. The Resection loss is defined as the dot product of the N and M. 

![equations](http://www.sciweavers.org/tex2img.php?eq=%5Cwidehat%7BN%7D%20%3D%20oa%20%5Cotimes%20ab%20%5C%5C%20%5Cwidehat%7BM%7D%20%3D%20R%20%5Ctimes%20AB%20%2B%20T%20%5C%5C%20E_%7Bj%7D%20%3D%20%5Cwidehat%7BN%7D%20%5Ccircledcirc%20%5Cwidehat%7BM%7D%20%5C%5CTotal%20loss%20%3D%20%5Csum_%7Bj%3D1%7D%5E%7BN%7DE_%7Bj%7D&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0[)
where N is the number of correspondences available.

