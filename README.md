# Cylinder Measurement

## Software and hardware used:
- Luxonis' DepthAI v3 (https://github.com/luxonis/depthai-core/tree/v3_develop) and Luxonis' Oak D PRO stereo depth camera
- Open3D (https://github.com/isl-org/open3d/tree/v0.19.0)
- Point Cloud Library v1.14.0


---


## 
### 1. Setup:
- Cloned, built and installed DepthAI v3 
- Installed other dependencies used, such as OpenCV, Open3D, Point Cloud Library
- CMake project setup

### 2. Saving a point cloud to a file using DepthAI and Oak camera:
- Expanded the visualizer_rgbd.cpp example (provided in the DepthAI examples folder) with the functionality of saving a
pointcloud to a file with a corresponding color image, by pressing a keyboard button
- Created a dataset of point cloud examples for three different cylinders, recorded from different angles 
(stored in the data folder of the repository)

### 3. Cylinder segmentation and measurement:
- Segmentation and measurement code is based on the: https://pointclouds.org/documentation/tutorials/cylinder_segmentation.html 

### 4. Segmentation and displaying the cylinder dimensions in real time:
- 


---

## Results, challenges and possible improvements:
- Results are ...
- To achieve a more robust cylinder segmentation use NN
- What about multiple cylinders? Limitations?