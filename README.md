# KinectFusion
## Introduction
Kinect Fusion builds a 3D model of a scene using the depth and color data recorded by an RGB-D camera.We have recorded our own data using the Intel RealSense depth camera.For the implementation we have followed the approach in "Kinectfusion: Real-time dense surface mapping and tracking" given by Izadi et al. with some modifications.
 
 ## Pipeline
![Pipeline](pipeline.jpg?raw=true "Optional Title")

- <b>Data Acquisition and Depth Map Conversion</b> <br />
Acquire the depth and color data from an RGB-D camera. The depth data from the sensor is back-projected into the camera space to create the vertex map. Normal map is created by taking the cross product of approximate tangent vectors at each pixel.

- <b> Pose Estimation</b> <br />
This step aims to estimate the 6DoF camera pose. To estimate the pose of each frame, we have used the linear least-square optimization of Iterative Closest Point algorithm with point to plane error metric and projective data association for correspondence finding. The estimated pose gives the transformation from camera to global space.

- <b>Volumetric Representation And Integration </b><br />
We have used a voxel grid system to represent a global volumetric model. Each voxel contains a truncated signed distance value which represents how far the corresponding voxel is from the surface.

- <b>Surface Prediction via Raycasting</b> <br />
Generates view of the implicit surface by rendering the surface at the zero-crossing. It provides a better estimate of global coordinates and normals for each frame.

- <b>Volume Visualization</b> <br />
In order to visualize the fused volume the Marching Cubes algorithm is used.

## Requirements
- Install FreeImage <br />
 sudo apt-get install libfreeimage3 libfreeimage-dev

- Install Ceres and dependencies <br />
 sudo apt-get install libeigen3-dev <br />
 sudo apt-get install libgoogle-glog-dev <br />
 sudo apt-get install libceres-dev <br />
