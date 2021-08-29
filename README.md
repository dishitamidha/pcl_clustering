# pcl_clustering

### [link to c++ file](src/main.cpp)
## Frameworks used
* ROS Melodic
* PCL 1.8

## Methods used
* Voxel Grid Filtering
* RANSAC Planar Segmentation (ground plane removal)
* Extracting Indices
* Creating KD Tree object for search method
* Euclidean Cluster Extraction

<table>
  <tr>
    <td>Raw Point Cloud</td>
    <td>Point Cloud after voxelizing, ground plane removal and clustering</td>
  <tr>
    <td><img src="https://user-images.githubusercontent.com/75261680/131235150-4075159e-63b2-4c29-b9d5-c88d6881f122.png" width=600></td>
    <td><img src="https://user-images.githubusercontent.com/75261680/131235151-7e8cfc58-bfba-4ff1-a7d8-1b92879af31e.png" width=600></td>
  </tr>
 </table>
 
 <table>
  <tr>
    <td>Less Cluttered Environment</td>
    <td>More Cluttered Environment</td>
  <tr>
    <td><img src="https://user-images.githubusercontent.com/75261680/131235287-bd949903-a33d-40c8-9c36-d689f5603f6c.gif" width=600></td>
    <td><img src="https://user-images.githubusercontent.com/75261680/131235285-07382d72-0b3d-451f-be91-8f0b10263022.gif" width=600></td>
  </tr>
 </table>


###  Clone Repository
``` 
git clone https://github.com/dishitamidha/pcl_task.git
```

###  Build Workspace
``` 
catkin_make 
```

###  Launch Simulation
``` 
roslaunch pcl_task pcl.launch 
```

###  Run PCL clustering node
```
rosrun pcl_task pcl_task_main 
```




